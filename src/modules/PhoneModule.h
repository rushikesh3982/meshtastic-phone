#pragma once
/*
 * T-Deck phone: the 4G modem (a SIMCom A7672S on a KTRON board), on the T-Deck's Grove pins.
 *
 * Built only with -D TDECK_PHONE (the t-deck-phone environment). The modem's power has its own switch, so it can come
 * and go at any time: the module notices, and sets it up again when it comes back.
 *
 * The module asks the modem one question at a time and never waits for an answer: it checks for the reply on its
 * next turns, so the rest of Meshtastic keeps running. Calls, rings and SMS arrive as the modem's own messages.
 *
 * Project home: https://github.com/rushikesh3982/tdeck-phone
 */
#include "configuration.h"

#ifdef TDECK_PHONE

#include "Observer.h"
#include "concurrency/Lock.h"
#include "concurrency/OSThread.h"
#include <esp_sleep.h>
#include <stdint.h>

// PhoneStatus and IPhone are shared with the touch interface (rushikesh3982/device-ui, include/phone/IPhone.h).
#include "phone/IPhone.h"

class PhoneModule : public IPhone, private concurrency::OSThread
{
  public:
    PhoneModule();

    // A copy of the current state. Safe from any task.
    PhoneStatus getStatus() override;

    // Calls. Safe from any task: the command is queued, and sent on the module's next turn.
    bool dial(const char *number) override;
    bool answer() override;
    bool hangUp() override;

    // SMS, newest first. Safe from any task. Received ones live on the SIM; sent ones only until restart.
    uint8_t smsCount() override;
    bool smsGet(uint8_t position, PhoneSms &out) override;
    bool sendSms(const char *number, const char *text) override;
    bool deleteSms(uint8_t position) override;
    void markSmsRead(uint8_t position) override;
    uint16_t smsVersion() override;

  protected:
    int32_t runOnce() override;

  private:
    enum class Job : uint8_t {
        None,
        Alive,
        Setup,
        Model,
        Sim,
        Operator,
        Registration,
        Volte,
        Signal,
        Battery,
        Gnss,
        Storage,
        User,
        SmsList,
        SmsRead,
        SmsDelete,
        SmsPrompt, // AT+CMGS sent, waiting for the modem's "> "
        SmsBody    // the text sent, waiting for OK
    };

    concurrency::Lock lock; // guards status and the queue
    PhoneStatus status;

    static constexpr int QUEUE_LEN = 4;
    char queue[QUEUE_LEN][32] = {};
    uint8_t queueHead = 0, queueCount = 0;
    bool queueCommand(const char *cmd);

    // the command waiting for its reply
    Job job = Job::None;
    char sent[40] = "";
    uint32_t sentAt = 0;
    uint32_t timeoutMs = 0;
    char reply[320] = "";
    char line[320] = "";
    uint16_t lineLen = 0;

    bool serialStarted = false;
    bool needSetup = true;
    uint8_t setupStep = 0;
    uint8_t pollStep = 0;
    uint8_t missedAts = 0;
    uint32_t nextPollAt = 0;
    bool gnssReady = false;
    uint32_t gnssStartedAt = 0;
    bool gnssRestart = false;
    bool ringtonePlaying = false;
    bool tonePlaying = false;

    // Sleep and battery
    int onPreflightSleep(void *);
    int onLightSleep(void *);
    int onLightSleepEnd(esp_sleep_wakeup_cause_t cause);
    int onDeepSleep(void *);
    CallbackObserver<PhoneModule, void *> preflightObserver =
        CallbackObserver<PhoneModule, void *>(this, &PhoneModule::onPreflightSleep);
    CallbackObserver<PhoneModule, void *> lightSleepObserver =
        CallbackObserver<PhoneModule, void *>(this, &PhoneModule::onLightSleep);
    CallbackObserver<PhoneModule, esp_sleep_wakeup_cause_t> lightSleepEndObserver =
        CallbackObserver<PhoneModule, esp_sleep_wakeup_cause_t>(this, &PhoneModule::onLightSleepEnd);
    CallbackObserver<PhoneModule, void *> deepSleepObserver =
        CallbackObserver<PhoneModule, void *>(this, &PhoneModule::onDeepSleep);
    bool wakeOnModem = false;
    bool smsUnreadCheckWanted = false;
    uint32_t nextBatteryCheckAt = 0;
    uint8_t criticalReadings = 0;
    void checkBattery();

    // SMS
    static constexpr uint8_t SMS_MAX = 30;   // held in RAM, newest first
    static constexpr uint8_t SMS_QUEUE = 6;
    PhoneSms smsList[SMS_MAX];
    uint8_t smsHeld = 0;
    uint16_t smsChanges = 0;
    bool smsListWanted = false;
    int16_t smsToRead[SMS_QUEUE] = {};
    uint8_t smsToReadCount = 0;
    int16_t smsToDelete[SMS_QUEUE] = {};
    uint8_t smsToDeleteCount = 0;
    bool smsOutPending = false;
    char smsOutNumber[24] = "";
    char smsOutText[161] = "";
    char smsUnreadTimes[SMS_MAX][24] = {}; // unread messages, remembered while the SIM is read again in full
    uint8_t smsUnreadTimesCount = 0;
    uint8_t unreadBeforeList = 0; // to chirp when a list (after a sleep) brings new messages
    PhoneSms smsBuilding;
    bool smsBuildingActive = false;
    int16_t smsReadingIndex = -1;

    void ring();
    void playTone(const char *rtttl, size_t len);
    void smsHeader(const char *l);
    void smsFlush();
    void smsKeep(const PhoneSms &m); // with the lock held
    void smsRecount();               // with the lock held
    void smsPromptArrived();

    void send(Job j, const char *cmd, uint32_t timeout);
    void finish(bool ok);
    void startNextJob();
    void readModem();
    void onLine(const char *l);
    void handleEvent(const char *l);
    void modemLost();
    bool gnssWanted();
    void parseGnss(const char *l);
    void publishFix(double lat, double lon, float altitude, uint8_t satellites, float hdop, float pdop, const char *date,
                    const char *utc);
};

extern PhoneModule *phoneModule;

#endif
