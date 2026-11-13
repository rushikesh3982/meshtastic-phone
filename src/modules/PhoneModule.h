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

#include "concurrency/Lock.h"
#include "concurrency/OSThread.h"
#include <stdint.h>

// What the phone knows about the modem. The touch interface reads a copy (PhoneModule::getStatus).
struct PhoneStatus {
    bool modemAlive = false; // the modem answers
    char model[24] = "";     // e.g. A7672S-FASE

    enum Sim : uint8_t { SIM_UNKNOWN, SIM_READY, SIM_NEEDS_PIN, SIM_MISSING };
    Sim sim = SIM_UNKNOWN;

    int8_t registration = -1; // +CEREG: 1 home, 5 roaming, 2 searching, 3 refused, 0 not searching, -1 unknown
    char operatorName[24] = "";
    bool volte = false;       // registered for calls over 4G
    bool lte = false;         // on 4G now
    int16_t rsrpDbm = 0;      // 4G signal, 0 = unknown
    int16_t band = -1;
    uint16_t modemMillivolts = 0;

    bool gnssOn = false;
    bool gnssFix = false;
    uint8_t satellites = 0;

    enum Call : uint8_t { CALL_IDLE, CALL_RINGING, CALL_DIALING, CALL_ACTIVE };
    Call call = CALL_IDLE;
    char callNumber[24] = ""; // the other party, while ringing or in a call

    int16_t newSmsIndex = -1; // the index of the latest SMS that arrived, -1 = none
};

class PhoneModule : private concurrency::OSThread
{
  public:
    PhoneModule();

    // A copy of the current state. Safe from any task.
    PhoneStatus getStatus();

    // Calls. Safe from any task: the command is queued, and sent on the module's next turn.
    bool dial(const char *number);
    bool answer();
    bool hangUp();

  protected:
    int32_t runOnce() override;

  private:
    enum class Job : uint8_t { None, Alive, Setup, Model, Sim, Operator, Registration, Volte, Signal, Battery, Gnss, User };

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
