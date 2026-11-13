#include "configuration.h"

#ifdef TDECK_PHONE

#include "PhoneModule.h"
#include "GPSStatus.h"
#include "NodeDB.h"
#include "PowerFSM.h"
#include "PowerStatus.h"
#include "sleep.h"
#include <driver/gpio.h>
#include "concurrency/LockGuard.h"
#include "gps/RTC.h"
#include <Arduino.h>
#include <math.h>
#if !MESHTASTIC_EXCLUDE_GPS
#include "modules/PositionModule.h"
#endif
#ifdef HAS_I2S
#include "main.h" // audioThread: the T-Deck's speaker
#endif

#ifndef PHONE_MODEM_RX_PIN
#error "TDECK_PHONE needs PHONE_MODEM_RX_PIN and PHONE_MODEM_TX_PIN (see variants/esp32s3/t-deck/variant.h)"
#endif

PhoneModule *phoneModule;

namespace
{
constexpr uint32_t BAUD = 115200;
constexpr uint32_t POLL_GAP_MS = 1500;   // one status question this often
constexpr uint32_t ALIVE_GAP_MS = 2000;  // while the modem is off: one AT this often
constexpr uint8_t LOST_AFTER = 3;        // unanswered ATs before the modem counts as off
constexpr uint32_t GNSS_RETRY_MS = 90000; // GNSS off and on again if it hasn't said READY by then
constexpr uint8_t POLL_STEPS = 9;
constexpr uint32_t BATTERY_CHECK_MS = 10000;
constexpr int BATTERY_LOW_MV = 3400;      // a warning on the screen
constexpr int BATTERY_CRITICAL_MV = 3300; // the modem is switched off (as the bench docs planned with AT+CPMVT)
constexpr uint8_t CRITICAL_READINGS = 3;  // in a row, so one dip during a transmit burst doesn't count

// The ringtone (RTTTL): a plain double ring, then a pause. The modem rings again every few seconds, and it repeats.
const char RINGTONE[] = "ring:d=16,o=6,b=200:e,c,e,c,e,c,e,c,4p,e,c,e,c,e,c,e,c,1p";
// A short rising chirp for a new SMS.
const char SMS_TONE[] = "sms:d=16,o=6,b=180:c,e,g,8c7";

// Lines the modem sends by itself, not as a reply to a question.
const char *const EVENTS[] = {"RING",        "+CLIP:", "VOICE CALL:", "NO CARRIER", "BUSY",     "NO ANSWER",
                              "MISSED_CALL", "+CMTI:", "+CGNSSPWR:",  "*ATREADY",   "SMS DONE", "PB DONE"};

bool validNumber(const char *number)
{
    size_t len = number ? strlen(number) : 0;
    if (len == 0 || len > 20)
        return false;
    for (size_t i = 0; i < len; i++) {
        char c = number[i];
        if (!isdigit((unsigned char)c) && c != '+' && c != '*' && c != '#')
            return false;
    }
    return true;
}

// The fields after a reply's colon, split at commas outside quotes, quotes removed, each cut to 27 characters.
// For SMS headers, whose time ("26/11/13,10:30:00+22") has a comma inside its quotes.
int quotedFields(const char *l, char out[][28], int maxFields)
{
    const char *p = strchr(l, ':');
    p = p ? p + 1 : l;
    while (*p == ' ')
        p++;
    int n = 0;
    size_t len = 0;
    bool inQuotes = false;
    out[0][0] = 0;
    for (;; p++) {
        char c = *p;
        if (c == '"') {
            inQuotes = !inQuotes;
            continue;
        }
        if (c == 0 || (c == ',' && !inQuotes)) {
            out[n][len] = 0;
            n++;
            len = 0;
            if (c == 0 || n >= maxFields)
                break;
            out[n][0] = 0;
            continue;
        }
        if (len < 27)
            out[n][len++] = c;
    }
    return n;
}

// Now, in the same form the network gives an SMS's time (yy/MM/dd,hh:mm:ss), from the T-Deck's clock.
void timeNow(char *out, size_t len)
{
    uint32_t now = getValidTime(RTCQualityFromNet, true);
    out[0] = 0;
    if (!now)
        return;
    time_t t = now;
    struct tm tm;
    gmtime_r(&t, &tm);
    strftime(out, len, "%y/%m/%d,%H:%M:%S", &tm);
}

bool startsWith(const char *s, const char *prefix)
{
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

bool isEvent(const char *l)
{
    for (const char *e : EVENTS) {
        if (startsWith(l, e))
            return true;
    }
    return false;
}

// The reply line that starts with prefix, copied into out.
bool replyLine(const char *reply, const char *prefix, char *out, size_t outLen)
{
    const char *at = strstr(reply, prefix);
    if (!at)
        return false;
    const char *end = strchr(at, '\n');
    size_t len = end ? (size_t)(end - at) : strlen(at);
    if (len >= outLen)
        len = outLen - 1;
    memcpy(out, at, len);
    out[len] = 0;
    return true;
}

// The fields after a reply's colon, split at commas, each cut to 19 characters.
int fields(const char *l, char out[][20], int maxFields)
{
    const char *p = strchr(l, ':');
    p = p ? p + 1 : l;
    int n = 0;
    while (n < maxFields) {
        while (*p == ' ')
            p++;
        const char *e = strchr(p, ',');
        size_t len = e ? (size_t)(e - p) : strlen(p);
        size_t keep = len < 19 ? len : 19;
        memcpy(out[n], p, keep);
        out[n][keep] = 0;
        while (keep && out[n][keep - 1] == ' ')
            out[n][--keep] = 0;
        n++;
        if (!e)
            break;
        p = e + 1;
    }
    return n;
}

void quoted(const char *l, char *out, size_t outLen)
{
    out[0] = 0;
    const char *a = strchr(l, '"');
    if (!a)
        return;
    const char *b = strchr(a + 1, '"');
    if (!b)
        return;
    size_t len = (size_t)(b - a - 1);
    if (len >= outLen)
        len = outLen - 1;
    memcpy(out, a + 1, len);
    out[len] = 0;
}

// CPSI's RSRP: this board's firmware gave 0..97 (dBm + 140) on the bench; other firmware gives dBm, or tenths of one.
int16_t rsrpDbm(int v)
{
    if (v >= 0 && v <= 97)
        return v - 140;
    if (v < -200)
        return v / 10;
    return v;
}

// A coordinate in decimal degrees, or in NMEA's ddmm.mmmm (SIM7600-style firmware).
double toDegrees(double v, double limit)
{
    if (fabs(v) <= limit)
        return v;
    double deg = floor(fabs(v) / 100);
    double result = deg + (fabs(v) - deg * 100) / 60;
    return v < 0 ? -result : result;
}

int twoDigits(const char *s)
{
    return (s[0] - '0') * 10 + (s[1] - '0');
}
} // namespace

PhoneModule::PhoneModule() : concurrency::OSThread("Phone")
{
    IPhone::set(this); // the touch interface's phone screens use this module
    preflightObserver.observe(&preflightSleep);
    lightSleepObserver.observe(&notifyLightSleep);
    lightSleepEndObserver.observe(&notifyLightSleepEnd);
    deepSleepObserver.observe(&notifyDeepSleep);
}

// Meshtastic asks before each light sleep: no sleep during a call, while a ringtone plays or an SMS is being sent.
int PhoneModule::onPreflightSleep(void *)
{
    concurrency::LockGuard guard(&lock);
    bool busy = status.call != PhoneStatus::CALL_IDLE || status.smsSend == PhoneStatus::SMS_SENDING || smsOutPending ||
                job == Job::SmsPrompt || job == Job::SmsBody || ringtonePlaying || tonePlaying;
    return busy ? 1 : 0;
}

// Just before a light sleep.
int PhoneModule::onLightSleep(void *)
{
    // Wake when the modem starts talking: its line idles high, and the first start bit pulls it low. The characters
    // that arrive while the T-Deck wakes up are lost: the modem repeats RING every few seconds, and new SMS are checked
    // for again after every wake. If the line is already low (the modem's side unpowered), it isn't used, or the
    // T-Deck would wake straight away again and again.
    wakeOnModem = serialStarted && digitalRead(PHONE_MODEM_RX_PIN) == HIGH;
    if (wakeOnModem)
        gpio_wakeup_enable((gpio_num_t)PHONE_MODEM_RX_PIN, GPIO_INTR_LOW_LEVEL);
#ifdef KB_POWERON
    // The peripherals' power stays on: it also feeds the modem board's VDD (its level shifters) through the Grove 3.3 V
    gpio_hold_en((gpio_num_t)KB_POWERON);
#endif
    return 0;
}

int PhoneModule::onLightSleepEnd(esp_sleep_wakeup_cause_t cause)
{
    if (wakeOnModem)
        gpio_wakeup_disable((gpio_num_t)PHONE_MODEM_RX_PIN);
    wakeOnModem = false;
#ifdef KB_POWERON
    gpio_hold_dis((gpio_num_t)KB_POWERON);
#endif
    smsUnreadCheckWanted = true; // an SMS that arrived during the sleep would have been missed
    return 0;
}

// The T-Deck is shutting down, or going to deep sleep for a critically low battery: switch the modem off too. It has
// its own switch, so it would otherwise go on draining the cell.
int PhoneModule::onDeepSleep(void *)
{
    if (serialStarted) {
        Serial1.print("AT+CPOF\r");
        Serial1.flush();
        delay(300);
    }
    return 0;
}

// Every 10 s: a warning below 3.4 V, and below 3.3 V (three readings in a row) the modem is switched off cleanly
// (AT+CPOF) before the cell runs flat. The modem's own reading is used when it looks right (it is taken at the modem's
// supply); on the bench it read about 0.01 V on 5 V, so otherwise the T-Deck's reading is used.
void PhoneModule::checkBattery()
{
    if ((int32_t)(millis() - nextBatteryCheckAt) < 0)
        return;
    nextBatteryCheckAt = millis() + BATTERY_CHECK_MS;
    if (!powerStatus || !powerStatus->getHasBattery())
        return;
    bool onUsb = powerStatus->getHasUSB();
    uint16_t modemMv;
    bool alive;
    {
        concurrency::LockGuard guard(&lock);
        modemMv = status.modemMillivolts;
        alive = status.modemAlive;
    }
    int mv = (modemMv >= 2800 && modemMv <= 4500) ? modemMv : powerStatus->getBatteryVoltageMv();
    if (mv <= 0)
        return;
    bool critical = !onUsb && mv < BATTERY_CRITICAL_MV;
    criticalReadings = critical ? criticalReadings + 1 : 0;
    {
        concurrency::LockGuard guard(&lock);
        status.batteryLow = !onUsb && mv < BATTERY_LOW_MV;
    }
    if (alive && criticalReadings >= CRITICAL_READINGS) {
        criticalReadings = 0;
        LOG_WARN("Phone: battery at %d mV: switching the modem off", mv);
        if (queueCommand("AT+CPOF")) {
            concurrency::LockGuard guard(&lock);
            status.modemOffForBattery = true;
        }
    }
}

PhoneStatus PhoneModule::getStatus()
{
    concurrency::LockGuard guard(&lock);
    return status;
}

bool PhoneModule::queueCommand(const char *cmd)
{
    concurrency::LockGuard guard(&lock);
    if (queueCount >= QUEUE_LEN)
        return false;
    strlcpy(queue[(queueHead + queueCount) % QUEUE_LEN], cmd, sizeof queue[0]);
    queueCount++;
    return true;
}

bool PhoneModule::dial(const char *number)
{
    if (!validNumber(number))
        return false;
    char cmd[32];
    snprintf(cmd, sizeof cmd, "ATD%s;", number);
    if (!queueCommand(cmd))
        return false;
    concurrency::LockGuard guard(&lock);
    status.call = PhoneStatus::CALL_DIALING;
    strlcpy(status.callNumber, number, sizeof status.callNumber);
    LOG_INFO("Phone: dialling");
    return true;
}

bool PhoneModule::answer()
{
    return queueCommand("ATA");
}

bool PhoneModule::hangUp()
{
    return queueCommand("AT+CHUP");
}

uint8_t PhoneModule::smsCount()
{
    concurrency::LockGuard guard(&lock);
    return smsHeld;
}

bool PhoneModule::smsGet(uint8_t position, PhoneSms &out)
{
    concurrency::LockGuard guard(&lock);
    if (position >= smsHeld)
        return false;
    out = smsList[position];
    return true;
}

uint16_t PhoneModule::smsVersion()
{
    concurrency::LockGuard guard(&lock);
    return smsChanges;
}

void PhoneModule::markSmsRead(uint8_t position)
{
    concurrency::LockGuard guard(&lock);
    if (position < smsHeld && smsList[position].unread) {
        smsList[position].unread = false;
        smsRecount();
        smsChanges++;
    }
}

bool PhoneModule::deleteSms(uint8_t position)
{
    concurrency::LockGuard guard(&lock);
    if (position >= smsHeld)
        return false;
    int16_t index = smsList[position].index;
    if (index >= 0) { // stored on the SIM: delete it there too
        if (smsToDeleteCount >= SMS_QUEUE)
            return false;
        smsToDelete[smsToDeleteCount++] = index;
    }
    memmove(&smsList[position], &smsList[position + 1], (smsHeld - position - 1) * sizeof(PhoneSms));
    smsHeld--;
    smsRecount();
    smsChanges++;
    return true;
}

bool PhoneModule::sendSms(const char *number, const char *text)
{
    if (!validNumber(number) || !text || !text[0])
        return false;
    concurrency::LockGuard guard(&lock);
    if (smsOutPending || status.smsSend == PhoneStatus::SMS_SENDING)
        return false;
    strlcpy(smsOutNumber, number, sizeof smsOutNumber);
    // Plain text only: anything else would need the modem's UCS2 mode. Ctrl-Z and Esc would end the message early.
    size_t n = 0;
    for (const char *p = text; *p && n < sizeof smsOutText - 1; p++) {
        unsigned char c = (unsigned char)*p;
        smsOutText[n++] = (c == '\n' || (c >= 32 && c < 127)) ? (char)c : '?';
    }
    smsOutText[n] = 0;
    smsOutPending = true;
    status.smsSend = PhoneStatus::SMS_SENDING;
    LOG_INFO("Phone: sending an SMS");
    return true;
}

// With the lock held.
void PhoneModule::smsRecount()
{
    uint8_t unread = 0;
    for (uint8_t i = 0; i < smsHeld; i++) {
        if (smsList[i].incoming && smsList[i].unread)
            unread++;
    }
    status.unreadSms = unread;
}

// With the lock held: adds a message, newest first, replacing one with the same place on the SIM.
void PhoneModule::smsKeep(const PhoneSms &message)
{
    PhoneSms m = message;
    if (m.index >= 0) {
        for (uint8_t i = 0; i < smsHeld; i++) {
            if (smsList[i].index == m.index) {
                if (smsList[i].unread && !strcmp(smsList[i].time, m.time))
                    m.unread = true; // still unread here, though reading it marked it read on the SIM
                memmove(&smsList[i], &smsList[i + 1], (smsHeld - i - 1) * sizeof(PhoneSms));
                smsHeld--;
                break;
            }
        }
        for (uint8_t i = 0; i < smsUnreadTimesCount && !m.unread; i++) {
            if (m.incoming && m.time[0] && !strcmp(smsUnreadTimes[i], m.time))
                m.unread = true;
        }
    }
    uint8_t at = 0;
    while (at < smsHeld && strcmp(smsList[at].time, m.time) >= 0)
        at++;
    if (smsHeld == SMS_MAX) {
        if (at >= SMS_MAX)
            return; // older than everything held
        smsHeld--; // the oldest goes
    }
    memmove(&smsList[at + 1], &smsList[at], (smsHeld - at) * sizeof(PhoneSms));
    smsList[at] = m;
    smsHeld++;
    smsRecount();
    smsChanges++;
}

// A message's header: +CMGL: <index>,<stat>,<number>,<name>,<time> or +CMGR: <stat>,<number>,<name>,<time>.
void PhoneModule::smsHeader(const char *l)
{
    char f[6][28];
    int n = quotedFields(l, f, 6);
    PhoneSms m;
    int o = 0;
    if (startsWith(l, "+CMGL:")) {
        m.index = (int16_t)atoi(f[0]);
        o = 1;
    } else {
        m.index = smsReadingIndex;
    }
    if (n < o + 2 || strstr(f[o], "UNSENT"))
        return; // a draft: not shown
    m.incoming = startsWith(f[o], "REC");
    m.unread = !strcmp(f[o], "REC UNREAD");
    strlcpy(m.number, f[o + 1], sizeof m.number);
    if (n > o + 3)
        strlcpy(m.time, f[o + 3], sizeof m.time);
    smsBuilding = m;
    smsBuildingActive = true;
}

void PhoneModule::smsFlush()
{
    if (!smsBuildingActive)
        return;
    smsBuildingActive = false;
    concurrency::LockGuard guard(&lock);
    smsKeep(smsBuilding);
}

// The modem asks for the text ("> "): send it, ended with Ctrl-Z.
void PhoneModule::smsPromptArrived()
{
    char text[sizeof smsOutText];
    {
        concurrency::LockGuard guard(&lock);
        strlcpy(text, smsOutText, sizeof text);
    }
    Serial1.print(text);
    Serial1.write((uint8_t)0x1A);
    job = Job::SmsBody;
    sentAt = millis();
    timeoutMs = 60000; // the network can take a while
    reply[0] = 0;
}

void PhoneModule::playTone(const char *rtttl, size_t len)
{
#ifdef HAS_I2S
    bool idle;
    {
        concurrency::LockGuard guard(&lock);
        idle = status.call == PhoneStatus::CALL_IDLE;
    }
    if (audioThread && idle && !ringtonePlaying) {
        audioThread->beginRttl(rtttl, len);
        tonePlaying = true;
    }
#endif
}

bool PhoneModule::gnssWanted()
{
    // The modem's GNSS is the phone's GPS: on unless GPS is switched off in Meshtastic's settings.
    return config.position.gps_mode != meshtastic_Config_PositionConfig_GpsMode_DISABLED;
}

int32_t PhoneModule::runOnce()
{
    if (!serialStarted) {
        Serial1.setRxBufferSize(2048);
        Serial1.begin(BAUD, SERIAL_8N1, PHONE_MODEM_RX_PIN, PHONE_MODEM_TX_PIN);
        gpio_pullup_en((gpio_num_t)PHONE_MODEM_RX_PIN); // a defined level when the modem is off, for the sleep wake
        serialStarted = true;
        nextPollAt = millis() + 1000;
        LOG_INFO("Phone: modem port open (RX %d, TX %d)", PHONE_MODEM_RX_PIN, PHONE_MODEM_TX_PIN);
    }

    readModem();
    if (job != Job::None && millis() - sentAt > timeoutMs)
        finish(false);
    if (job == Job::None)
        startNextJob();
    ring();
    checkBattery();
#ifdef HAS_I2S
    if (tonePlaying && (!audioThread || !audioThread->isPlaying())) // isPlaying() also advances the tone
        tonePlaying = false;
#endif
    return (job == Job::None && !ringtonePlaying && !tonePlaying) ? 50 : 20;
}

// The ringtone on the T-Deck's speaker while a call rings. The player only advances when asked (isPlaying), so this
// runs every 20 ms while ringing.
void PhoneModule::ring()
{
#ifdef HAS_I2S
    if (!audioThread)
        return;
    bool ringing;
    {
        concurrency::LockGuard guard(&lock);
        ringing = status.call == PhoneStatus::CALL_RINGING;
    }
    if (ringing) {
        if (!audioThread->isPlaying())
            audioThread->beginRttl(RINGTONE, strlen(RINGTONE));
        ringtonePlaying = true;
    } else if (ringtonePlaying) {
        audioThread->stop();
        ringtonePlaying = false;
    }
#endif
}

void PhoneModule::send(Job j, const char *cmd, uint32_t timeout)
{
    job = j;
    strlcpy(sent, cmd, sizeof sent);
    reply[0] = 0;
    sentAt = millis();
    timeoutMs = timeout;
    Serial1.print(cmd);
    Serial1.print("\r");
}

void PhoneModule::readModem()
{
    while (Serial1.available()) {
        char c = Serial1.read();
        if (job == Job::SmsPrompt && c == '>' && lineLen == 0) {
            smsPromptArrived();
            continue;
        }
        if (c == '\n') {
            while (lineLen && line[lineLen - 1] == ' ')
                lineLen--;
            line[lineLen] = 0;
            if (lineLen)
                onLine(line);
            lineLen = 0;
        } else if (c != '\r' && lineLen < sizeof line - 1) {
            line[lineLen++] = c;
        }
    }
}

void PhoneModule::onLine(const char *l)
{
    bool event = isEvent(l);
    if (event)
        handleEvent(l);
    if (job == Job::None)
        return;
    if (job == Job::User && (!strcmp(l, "NO CARRIER") || !strcmp(l, "BUSY") || !strcmp(l, "NO ANSWER"))) {
        finish(false);
        return;
    }
    if (event)
        return;
    if (job == Job::SmsList || job == Job::SmsRead) { // a message's text can run over several lines
        if (startsWith(l, "+CMGL:") || startsWith(l, "+CMGR:")) {
            smsFlush();
            smsHeader(l);
        } else if (!strcmp(l, "OK")) {
            smsFlush();
            finish(true);
        } else if (!strcmp(l, "ERROR") || startsWith(l, "+CME ERROR") || startsWith(l, "+CMS ERROR")) {
            smsBuildingActive = false;
            strlcpy(reply, l, sizeof reply);
            finish(false);
        } else if (smsBuildingActive) {
            char *t = smsBuilding.text;
            size_t used = strlen(t);
            if (used && used < sizeof smsBuilding.text - 1)
                t[used++] = '\n';
            for (const char *p = l; *p && used < sizeof smsBuilding.text - 1; p++)
                t[used++] = ((unsigned char)*p >= 32 && (unsigned char)*p < 127) ? *p : '?';
            t[used] = 0;
        }
        return;
    }
    if (!strcmp(l, "OK")) {
        finish(true);
    } else if (!strcmp(l, "ERROR") || startsWith(l, "+CME ERROR") || startsWith(l, "+CMS ERROR")) {
        strlcpy(reply, l, sizeof reply);
        finish(false);
    } else if (strcmp(l, sent) != 0) { // not the modem's echo of the command
        size_t used = strlen(reply);
        if (used + strlen(l) + 2 < sizeof reply) {
            strcat(reply, l);
            strcat(reply, "\n");
        }
    }
}

void PhoneModule::handleEvent(const char *l)
{
    concurrency::LockGuard guard(&lock);
    if (!strcmp(l, "RING")) {
        if (status.call != PhoneStatus::CALL_RINGING && status.call != PhoneStatus::CALL_ACTIVE) {
            LOG_INFO("Phone: incoming call");
            powerFSM.trigger(EVENT_INPUT); // awake, as for a key press
        }
        if (status.call != PhoneStatus::CALL_ACTIVE)
            status.call = PhoneStatus::CALL_RINGING;
    } else if (startsWith(l, "+CLIP:")) {
        quoted(l, status.callNumber, sizeof status.callNumber);
    } else if (startsWith(l, "VOICE CALL: BEGIN")) {
        status.call = PhoneStatus::CALL_ACTIVE;
        LOG_INFO("Phone: call connected");
    } else if (startsWith(l, "VOICE CALL: END") || !strcmp(l, "NO CARRIER") || !strcmp(l, "BUSY") ||
               !strcmp(l, "NO ANSWER") || startsWith(l, "MISSED_CALL")) {
        if (status.call != PhoneStatus::CALL_IDLE)
            LOG_INFO("Phone: call ended");
        status.call = PhoneStatus::CALL_IDLE;
        status.callNumber[0] = 0;
    } else if (startsWith(l, "+CMTI:")) { // a new SMS, stored at this index: read it next
        const char *comma = strrchr(l, ',');
        if (comma && smsToReadCount < SMS_QUEUE)
            smsToRead[smsToReadCount++] = (int16_t)atoi(comma + 1);
        LOG_INFO("Phone: new SMS");
    } else if (startsWith(l, "+CGNSSPWR: READY")) {
        gnssReady = true;
        LOG_INFO("Phone: GNSS ready");
    } else if (startsWith(l, "*ATREADY")) {
        needSetup = true; // the modem has just started
        setupStep = 0;
        gnssReady = false;
    }
}

void PhoneModule::modemLost()
{
    concurrency::LockGuard guard(&lock);
    PhoneStatus off;
    strlcpy(off.model, status.model, sizeof off.model);
    off.unreadSms = status.unreadSms; // the messages held stay
    off.batteryLow = status.batteryLow;
    off.modemOffForBattery = status.modemOffForBattery;
    status = off;
    needSetup = true;
    setupStep = 0;
    gnssReady = false;
    LOG_INFO("Phone: modem not answering (is its switch off?)");
}

void PhoneModule::startNextJob()
{
    uint32_t now = millis();
    bool alive;
    char cmd[32] = "";
    {
        concurrency::LockGuard guard(&lock);
        alive = status.modemAlive;
        if (alive && queueCount) {
            strlcpy(cmd, queue[queueHead], sizeof cmd);
            queueHead = (queueHead + 1) % QUEUE_LEN;
            queueCount--;
        }
    }

    if (!alive) {
        if ((int32_t)(now - nextPollAt) >= 0) {
            nextPollAt = now + ALIVE_GAP_MS;
            send(Job::Alive, "AT", 800);
        }
        return;
    }
    if (cmd[0]) {
        send(Job::User, cmd, 10000);
        return;
    }

    if (needSetup) {
        // Echo off; the caller's number with each ring; SMS as text, announced when they arrive; call sound to the
        // earpiece, at the volume (5) and mic gain (4) found right on the bench; no data-connection reports (they would
        // only wake the T-Deck).
        static const char *const SETUP[] = {"ATE0",          "AT+CLIP=1",     "AT+CMGF=1",  "AT+CNMI=2,1,0,0,0",
                                            "AT+CSDVC=1",    "AT+COUTGAIN=5", "AT+CMICGAIN=4", "AT+CGEREP=0"};
        constexpr uint8_t N = sizeof SETUP / sizeof SETUP[0];
        if (setupStep < N) {
            send(Job::Setup, SETUP[setupStep++], 2000);
            return;
        }
        if (setupStep == N) {
            setupStep++;
            bool want = gnssWanted();
            {
                concurrency::LockGuard guard(&lock);
                status.gnssOn = want;
            }
            gnssReady = false;
            gnssStartedAt = now;
            send(Job::Setup, want ? "AT+CGNSSPWR=1" : "AT+CGNSSPWR=0", 3000);
            return;
        }
        if (setupStep == N + 1) {
            setupStep++;
            send(Job::Model, "AT+CGMM", 2000);
            return;
        }
        needSetup = false;
        smsListWanted = true; // then read the messages on the SIM
        LOG_INFO("Phone: modem set up");
    }

    // SMS work comes before the status questions
    {
        bool out;
        int16_t toDelete = -1, toRead = -1;
        char number[24] = "";
        {
            concurrency::LockGuard guard(&lock);
            out = smsOutPending;
            if (out) {
                smsOutPending = false;
                strlcpy(number, smsOutNumber, sizeof number);
            } else if (smsToDeleteCount) {
                toDelete = smsToDelete[0];
                memmove(smsToDelete, smsToDelete + 1, --smsToDeleteCount * sizeof smsToDelete[0]);
            } else if (smsToReadCount) {
                toRead = smsToRead[0];
                memmove(smsToRead, smsToRead + 1, --smsToReadCount * sizeof smsToRead[0]);
            }
        }
        char cmd[40];
        if (out) {
            snprintf(cmd, sizeof cmd, "AT+CMGS=\"%s\"", number);
            send(Job::SmsPrompt, cmd, 5000);
            return;
        }
        if (toDelete >= 0) {
            snprintf(cmd, sizeof cmd, "AT+CMGD=%d", toDelete);
            send(Job::SmsDelete, cmd, 5000);
            return;
        }
        if (toRead >= 0) {
            smsReadingIndex = toRead;
            snprintf(cmd, sizeof cmd, "AT+CMGR=%d", toRead);
            send(Job::SmsRead, cmd, 5000);
            return;
        }
        if (smsListWanted) {
            smsListWanted = false;
            smsUnreadCheckWanted = false;
            {
                // What's on the SIM is read again in full: forget the copies held, but remember which were unread
                // (reading them from the SIM marked them read there).
                concurrency::LockGuard guard(&lock);
                uint8_t kept = 0;
                smsUnreadTimesCount = 0;
                for (uint8_t i = 0; i < smsHeld; i++) {
                    if (smsList[i].index < 0)
                        smsList[kept++] = smsList[i];
                    else if (smsList[i].unread)
                        strlcpy(smsUnreadTimes[smsUnreadTimesCount++], smsList[i].time, sizeof smsUnreadTimes[0]);
                }
                smsHeld = kept;
                smsRecount();
                smsChanges++;
            }
            unreadBeforeList = 255; // no chirp for what was already there
            send(Job::SmsList, "AT+CMGL=\"ALL\"", 15000);
            return;
        }
        if (smsUnreadCheckWanted) { // after a sleep: any SMS that arrived meanwhile
            smsUnreadCheckWanted = false;
            {
                concurrency::LockGuard guard(&lock);
                unreadBeforeList = status.unreadSms;
            }
            send(Job::SmsList, "AT+CMGL=\"REC UNREAD\"", 15000);
            return;
        }
    }

    if ((int32_t)(now - nextPollAt) < 0)
        return;
    nextPollAt = now + POLL_GAP_MS;
    uint8_t step = pollStep;
    pollStep = (pollStep + 1) % POLL_STEPS;

    switch (step) {
    case 0:
        send(Job::Alive, "AT", 1500);
        break;
    case 1:
        send(Job::Sim, "AT+CPIN?", 2000);
        break;
    case 2:
        send(Job::Operator, "AT+COPS?", 3000);
        break;
    case 3:
        send(Job::Registration, "AT+CEREG?", 2000);
        break;
    case 4:
        send(Job::Volte, "AT+CIREG?", 2000);
        break;
    case 5:
        send(Job::Signal, "AT+CPSI?", 2000);
        break;
    case 6:
        send(Job::Battery, "AT+CBC", 2000);
        break;
    case 8:
        send(Job::Storage, "AT+CPMS?", 2000);
        break;
    case 7: {
        bool want = gnssWanted();
        bool on;
        {
            concurrency::LockGuard guard(&lock);
            on = status.gnssOn;
        }
        if (want != on) { // GPS switched on or off in Meshtastic's settings
            {
                concurrency::LockGuard guard(&lock);
                status.gnssOn = want;
                status.gnssFix = false;
            }
            gnssReady = false;
            gnssStartedAt = now;
            send(Job::Setup, want ? "AT+CGNSSPWR=1" : "AT+CGNSSPWR=0", 3000);
        } else if (!want) {
            // GPS is off: nothing to ask
        } else if (!gnssReady && now - gnssStartedAt > GNSS_RETRY_MS) {
            // As on the bench: when READY doesn't come, GNSS off, then on again on the next round
            gnssRestart = !gnssRestart;
            if (!gnssRestart)
                gnssStartedAt = now;
            send(Job::Setup, gnssRestart ? "AT+CGNSSPWR=0" : "AT+CGNSSPWR=1", 3000);
        } else {
            send(Job::Gnss, "AT+CGNSSINFO", 2000);
        }
        break;
    }
    }
}

void PhoneModule::finish(bool ok)
{
    Job j = job;
    job = Job::None;
    char l[96];
    char f[20][20];

    switch (j) {
    case Job::Alive: {
        bool was;
        {
            concurrency::LockGuard guard(&lock);
            was = status.modemAlive;
            if (ok) {
                status.modemAlive = true;
                if (!was)
                    status.modemOffForBattery = false; // switched on again
            }
        }
        if (ok) {
            missedAts = 0;
            if (!was) {
                LOG_INFO("Phone: modem answers");
                needSetup = true;
                setupStep = 0;
                nextPollAt = millis();
            }
        } else if (was && ++missedAts >= LOST_AFTER) {
            missedAts = 0;
            modemLost();
        }
        break;
    }
    case Job::Model:
        if (ok) {
            const char *end = strchr(reply, '\n');
            size_t len = end ? (size_t)(end - reply) : strlen(reply);
            concurrency::LockGuard guard(&lock);
            if (len >= sizeof status.model)
                len = sizeof status.model - 1;
            memcpy(status.model, reply, len);
            status.model[len] = 0;
        }
        break;
    case Job::Sim: {
        PhoneStatus::Sim sim = PhoneStatus::SIM_UNKNOWN;
        if (ok && strstr(reply, "READY"))
            sim = PhoneStatus::SIM_READY;
        else if (strstr(reply, "PIN") || strstr(reply, "PUK"))
            sim = PhoneStatus::SIM_NEEDS_PIN;
        else if (strstr(reply, "not inserted") || strstr(reply, "ERROR: 10") || strstr(reply, "REMOVED"))
            sim = PhoneStatus::SIM_MISSING;
        concurrency::LockGuard guard(&lock);
        status.sim = sim;
        break;
    }
    case Job::Operator:
        if (ok && replyLine(reply, "+COPS:", l, sizeof l)) {
            concurrency::LockGuard guard(&lock);
            quoted(l, status.operatorName, sizeof status.operatorName);
        }
        break;
    case Job::Registration:
        if (ok && replyLine(reply, "+CEREG:", l, sizeof l) && fields(l, f, 20) >= 2) {
            concurrency::LockGuard guard(&lock);
            status.registration = (int8_t)atoi(f[1]);
        }
        break;
    case Job::Volte:
        if (ok && replyLine(reply, "+CIREG:", l, sizeof l)) {
            int n = fields(l, f, 20);
            concurrency::LockGuard guard(&lock);
            status.volte = n >= 2 && atoi(f[1]) == 1;
        }
        break;
    case Job::Signal:
        if (ok && replyLine(reply, "+CPSI:", l, sizeof l)) {
            int n = fields(l, f, 20);
            concurrency::LockGuard guard(&lock);
            status.lte = n > 0 && startsWith(f[0], "LTE");
            if (status.lte && n >= 5) {
                for (int i = 0; i < n; i++) {
                    const char *b = strstr(f[i], "BAND");
                    if (b)
                        status.band = (int16_t)atoi(b + 4);
                }
                status.rsrpDbm = rsrpDbm(atoi(f[n - 3]));
            } else {
                status.rsrpDbm = 0;
            }
        }
        break;
    case Job::Battery:
        if (ok && replyLine(reply, "+CBC:", l, sizeof l)) {
            concurrency::LockGuard guard(&lock);
            status.modemMillivolts = (uint16_t)(atof(strchr(l, ':') + 1) * 1000);
        }
        break;
    case Job::Gnss:
        if (ok && replyLine(reply, "+CGNSSINFO:", l, sizeof l)) {
            gnssReady = true; // it answers, so READY was only missed
            parseGnss(l);
        }
        break;
    case Job::Storage:
        if (ok && replyLine(reply, "+CPMS:", l, sizeof l)) {
            char q[6][28];
            if (quotedFields(l, q, 6) >= 3) {
                concurrency::LockGuard guard(&lock);
                status.smsStored = (uint8_t)atoi(q[1]);
                status.smsCapacity = (uint8_t)atoi(q[2]);
            }
        }
        break;
    case Job::SmsRead:
        if (ok) {
            LOG_INFO("Phone: SMS received");
            playTone(SMS_TONE, strlen(SMS_TONE));
        }
        break;
    case Job::SmsList: {
        bool newOnes;
        {
            concurrency::LockGuard guard(&lock);
            smsUnreadTimesCount = 0;
            newOnes = ok && unreadBeforeList != 255 && status.unreadSms > unreadBeforeList;
            if (ok)
                LOG_INFO("Phone: %u SMS held", smsHeld);
        }
        if (newOnes)
            playTone(SMS_TONE, strlen(SMS_TONE));
        break;
    }
    case Job::SmsPrompt: // no "> " from the modem
    case Job::SmsBody: {
        if (j == Job::SmsPrompt)
            Serial1.write((uint8_t)0x1B); // Esc: cancel the message
        bool sentOk = ok && j == Job::SmsBody;
        LOG_INFO(sentOk ? "Phone: SMS sent" : "Phone: SMS not sent");
        concurrency::LockGuard guard(&lock);
        status.smsSend = sentOk ? PhoneStatus::SMS_SENT : PhoneStatus::SMS_FAILED;
        if (sentOk) { // kept until restart, so the conversation shows it
            PhoneSms m;
            m.incoming = false;
            strlcpy(m.number, smsOutNumber, sizeof m.number);
            strlcpy(m.text, smsOutText, sizeof m.text);
            timeNow(m.time, sizeof m.time);
            smsKeep(m);
        }
        break;
    }
    case Job::User:
        if (!ok) {
            LOG_WARN("Phone: call command not accepted");
            concurrency::LockGuard guard(&lock);
            if (status.call == PhoneStatus::CALL_DIALING) {
                status.call = PhoneStatus::CALL_IDLE;
                status.callNumber[0] = 0;
            }
        }
        break;
    default:
        break;
    }
    reply[0] = 0;
}

void PhoneModule::parseGnss(const char *l)
{
    // +CGNSSINFO: <mode>,<satellite counts...>,<lat>,<N/S>,<lon>,<E/W>,<date>,<UTC>,<alt>,<speed>,<course>,<PDOP>,
    // <HDOP>,<VDOP>. The number of satellite-count fields differs between firmware, so the latitude is found as the
    // first field with a decimal point. No fix: empty fields.
    char f[24][20];
    int n = fields(l, f, 24);
    int latAt = -1;
    unsigned sats = 0;
    for (int i = 1; i < n; i++) {
        if (strchr(f[i], '.')) {
            latAt = i;
            break;
        }
        sats += atoi(f[i]);
    }
    bool fix = latAt > 0 && latAt + 3 < n && (f[latAt + 1][0] == 'N' || f[latAt + 1][0] == 'S');
    {
        concurrency::LockGuard guard(&lock);
        if (fix != status.gnssFix)
            LOG_INFO(fix ? "Phone: GNSS fix" : "Phone: GNSS fix lost");
        status.gnssFix = fix;
        status.satellites = (uint8_t)(sats > 255 ? 255 : sats);
    }
    if (!fix)
        return;

    double lat = toDegrees(atof(f[latAt]), 90);
    double lon = toDegrees(atof(f[latAt + 2]), 180);
    if (f[latAt + 1][0] == 'S')
        lat = -lat;
    if (f[latAt + 3][0] == 'W')
        lon = -lon;
    float altitude = n > latAt + 6 ? atof(f[latAt + 6]) : 0;
    float pdop = n > latAt + 9 ? atof(f[latAt + 9]) : 0;
    float hdop = n > latAt + 10 ? atof(f[latAt + 10]) : 0;
    publishFix(lat, lon, altitude, (uint8_t)sats, hdop, pdop, n > latAt + 4 ? f[latAt + 4] : "",
               n > latAt + 5 ? f[latAt + 5] : "");
}

void PhoneModule::publishFix(double lat, double lon, float altitude, uint8_t satellites, float hdop, float pdop,
                             const char *date, const char *utc)
{
    meshtastic_Position p = meshtastic_Position_init_default;
    p.has_latitude_i = true;
    p.latitude_i = (int32_t)lround(lat * 1e7);
    p.has_longitude_i = true;
    p.longitude_i = (int32_t)lround(lon * 1e7);
    p.has_altitude = true;
    p.altitude = (int32_t)lroundf(altitude);
    p.sats_in_view = satellites;
    p.HDOP = (uint32_t)lroundf(hdop * 100);
    p.PDOP = (uint32_t)lroundf(pdop * 100);
    p.location_source = meshtastic_Position_LocSource_LOC_INTERNAL;
    p.fix_quality = 1;
    p.fix_type = 3;

    // The fix's own time (UTC): date ddmmyy, time hhmmss.s. It also sets the clock.
    if (strlen(date) == 6 && strlen(utc) >= 6) {
        struct tm t = {};
        t.tm_mday = twoDigits(date);
        t.tm_mon = twoDigits(date + 2) - 1;
        t.tm_year = 100 + twoDigits(date + 4);
        t.tm_hour = twoDigits(utc);
        t.tm_min = twoDigits(utc + 2);
        t.tm_sec = twoDigits(utc + 4);
        p.timestamp = gm_mktime(&t);
        perhapsSetRTC(RTCQualityGPS, t);
    }
    p.time = getValidTime(RTCQualityFromNet);

    // A fixed position set in Meshtastic's settings wins, as it does over a GPS.
    if (config.position.fixed_position)
        return;

    nodeDB->updatePosition(nodeDB->getNodeNum(), p, RX_SRC_LOCAL);
    const meshtastic::GPSStatus fixStatus(true, true, false, p);
    gpsStatus->updateStatus(&fixStatus);
#if !MESHTASTIC_EXCLUDE_GPS
    if (positionModule)
        positionModule->handleNewPosition();
#endif
}

#endif
