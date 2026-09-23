#pragma once
#include "OpenKNX.h"
#include "hardware.h"
#include "knxprod.h"
#include <INA238.h>
#ifdef OPENKNX_BPS_TEMPSENS_ADDR
  #include <PCT2075.h>
#endif

#define OPENKNX_BPS_FLASH_VERSION 0
#define OPENKNX_BPS_FLASH_MAGIC_WORD 2847103552

#define BUS_LOAD_UPDATE_RATE 1000
#define BUS_LOAD_MAX_BYTES_PER_SECOND 800

#define POWER_OK_THRESHOLD_VOLTAGE 28
#define CURRENT_MAX_THRESHOLD_MA 1280

#define CURRENT_OVERLOAD_THRESHOLD_MAX_MA 3000
#define CURRENT_OVERLOAD_THRESHOLD_SHORT_MA 2400
#define CURRENT_OVERLOAD_THRESHOLD_SHORT_TIME_MS 3000

#define MAX_CURRENT_SWITCH_PER_SECOND 5
#define POWER_SUPPLY_SWITCH_RECENT_RESET_MS 1000

#define HOLD_RESET_DELAY_MS 500

class BusPowerSupplyModule : public OpenKNX::Module
{
  public:
    BusPowerSupplyModule();
    ~BusPowerSupplyModule();

    void processInputKo(GroupObject &ko);
    void setup(bool configured);
    void loop(bool configured);

    void writeFlash() override;
    void readFlash(const uint8_t* data, const uint16_t size) override;
    uint16_t flashSize() override;

    void savePower() override;
    bool restorePower() override;
    void processBeforeRestart() override;

    const std::string name() override;
    const std::string version() override;

    float estimateBusLoad();

    void showHelp() override;
    bool processCommand(const std::string cmd, bool diagnoseKo) override;
    void runTestMode();

  private:
    void pwr1On();
    void pwr1Off();
    void pwr2On();
    void pwr2Off();

    void processSendValue(GroupObject& ko, Dpt dpt, bool send, uint8_t sendMinChangePercent, uint16_t sendMinChangeAbsolute, uint32_t sendCyclicTimeMS, uint32_t& cyclicSendTimer, float& lastSentValue, float currentValue, uint16_t checkMultiply = 1);

    INA238 _inaKnx = INA238(OPENKNX_BPS_CURRENT_KNX_INA_ADDR, &OPENKNX_GPIO_WIRE);
    INA238 _inaAux = INA238(OPENKNX_BPS_CURRENT_AUX_INA_ADDR, &OPENKNX_GPIO_WIRE);

#ifdef OPENKNX_BPS_TEMPSENS_ADDR
    PCT2075 _temperature = PCT2075(OPENKNX_BPS_TEMPSENS_ADDR, &OPENKNX_GPIO_WIRE);
#endif

    uint8_t _pwrActive = 0;
    bool _pwrErrorLogged = false;
    bool _pwr1Ok = false;
    bool _pwr2Ok = false;
    bool _busOk = true;
    bool _currentOk = true;
    bool _overcurrent = false;
    uint32_t _overcurrentStarted = 0;
    uint32_t _overcurrentShortStarted = 0;
    uint8_t _overcurrentRetryCount = 0;
    uint8_t _recentPwrSupplySwitches = 0;
    uint32_t _lastPwrSupplySwitch = 0;
    
    bool _pwrReadFromFlash = false;
    bool _firstLoop = true;
    uint32_t _holdResetDelay = 0;

    bool _resetActive = false;
    uint32_t _resetStarted = 0;

    float _lastPowerSupply1Sent = 0;
    float _lastPowerSupply2Sent = 0;
    float _lastBusVoltageSent = 0;
    float _lastBusCurrentSent = 0;
    float _lastBusLoadSent = 0;
    float _lastAuxVoltageSent = 0;
    float _lastAuxCurrentSent = 0;
    float _lastTemperatureSent = 0;
    uint32_t _powerSupply1SendTimer = 0;
    uint32_t _powerSupply2SendTimer = 0;
    uint32_t _busVoltageSendTimer = 0;
    uint32_t _busCurrentSendTimer = 0;
    uint32_t _busLoadSendTimer = 0;
    uint32_t _auxVoltageSendTimer = 0;
    uint32_t _auxCurrentSendTimer = 0;
    uint32_t _temperatureSendTimer = 0;

    uint32_t _busLoadUpdateTimer = 0;
    uint32_t _rxLastBusLoadTime;
    uint32_t _rxLastBusBytes;

    uint32_t _debugTimer = 0;
};

extern BusPowerSupplyModule openknxBusPowerSupplyModule;