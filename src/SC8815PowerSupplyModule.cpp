#include "SC8815PowerSupplyModule.h"
#include "OpenKNX.h"
#include "ModuleVersionCheck.h"

BusPowerSupplyModule openknxBusPowerSupplyModule;

BusPowerSupplyModule::BusPowerSupplyModule()
{
}

BusPowerSupplyModule::~BusPowerSupplyModule()
{
}

const std::string BusPowerSupplyModule::name()
{
    return "BusPower";
}

const std::string BusPowerSupplyModule::version()
{
    return MODULE_BusPowerSupply_Version;
}

/**
 * @brief 处理输入对象(GroupObject)的函数
 * @param ko 引用类型的GroupObject对象，包含需要处理的数据
 */
void BusPowerSupplyModule::processInputKo(GroupObject &ko) // 
{ 处理输入组对象
    logDebugP("processInputKo");  // 记录调试信息，表示进入processInputKo函数
    logIndentUp();  // 增加日志缩进，通常用于嵌套函数调用的日志格式化

    uint16_t asap = ko.asap();  // 获取组对象的ASAP(Access Point Address)值
    switch (asap)  // 根据ASAP值进行不同的处理
    {
    }  // 目前switch语句为空，可能需要后续扩展

    logIndentDown();  // 减少日志缩进，对应logIndentUp()
}

void BusPowerSupplyModule::setup(bool configured)
{
    openknx.gpio.pinMode(OPENKNX_BPS_PWR1_CHECK_PIN, INPUT);
    openknx.gpio.pinMode(OPENKNX_BPS_PWR1_ALERT_PIN, INPUT);

    openknx.gpio.pinMode(OPENKNX_BPS_PWR2_CHECK_PIN, INPUT);
    openknx.gpio.pinMode(OPENKNX_BPS_PWR2_ALERT_PIN, INPUT);

    openknx.gpio.pinMode(OPENKNX_BPS_STATUS_BUS, OUTPUT, true, !OPENKNX_BPS_STATUS_ACTIVE_ON);
    openknx.gpio.pinMode(OPENKNX_BPS_STATUS_TMP, OUTPUT, true, !OPENKNX_BPS_STATUS_ACTIVE_ON);
    openknx.gpio.pinMode(OPENKNX_BPS_STATUS_TRC, OUTPUT, true, !OPENKNX_BPS_STATUS_ACTIVE_ON);
    openknx.gpio.pinMode(OPENKNX_BPS_STATUS_DEV, OUTPUT, true, !OPENKNX_BPS_STATUS_ACTIVE_ON);
    openknx.gpio.pinMode(OPENKNX_BPS_STATUS_PW1, OUTPUT, true, !OPENKNX_BPS_STATUS_ACTIVE_ON);
    openknx.gpio.pinMode(OPENKNX_BPS_STATUS_PW2, OUTPUT, true, !OPENKNX_BPS_STATUS_ACTIVE_ON);
    openknx.gpio.pinMode(OPENKNX_BPS_STATUS_MAX, OUTPUT, true, !OPENKNX_BPS_STATUS_ACTIVE_ON);
    openknx.gpio.pinMode(OPENKNX_BPS_STATUS_RST, OUTPUT, true, !OPENKNX_BPS_STATUS_ACTIVE_ON);
    openknx.gpio.pinMode(OPENKNX_BPS_SWITCH_RST, INPUT);

    analogReadResolution(12);

    if (_inaKnx.begin())
    {
        logDebugP("KNX INA238 setup done with address %u", OPENKNX_BPS_CURRENT_KNX_INA_ADDR);
        logIndentUp();

        _inaKnx.setMaxCurrentShunt(3, OPENKNX_BPS_CURRENT_KNX_INA_SHUNT);
        _inaKnx.setAverage(INA238_16_SAMPLES);
        _inaKnx.setMode(INA238_MODE_CONT_TEMP_BUS_SHUNT);

#ifdef OPENKNX_DEBUG
        delay(1000);
        logDebugP("getMode %u", _inaKnx.getMode());
        // logDebugP("isCalibrated %u", _inaKnx.isCalibrated());
        // logDebugP("getCurrentLSB %.4f", _inaKnx.getCurrentLSB());
        // logDebugP("getShunt %.4f", _inaKnx.getShunt());
        // logDebugP("getMaxCurrent %.4f", _inaKnx.getMaxCurrent());
#endif
        logIndentDown();
    }
    else
        logDebugP("KNX INA238 not found at address %u", OPENKNX_BPS_CURRENT_KNX_INA_ADDR);

    if (_inaAux.begin())
    {
        logDebugP("AUX INA238 setup done with address %u", OPENKNX_BPS_CURRENT_AUX_INA_ADDR);
        logIndentUp();

        _inaAux.setMaxCurrentShunt(3, OPENKNX_BPS_CURRENT_KNX_INA_SHUNT);
        _inaAux.setAverage(INA238_16_SAMPLES);
        _inaAux.setMode(INA238_MODE_CONT_TEMP_BUS_SHUNT);

#ifdef OPENKNX_DEBUG
        delay(1000);
        logDebugP("getMode %u", _inaAux.getMode());
        // logDebugP("isCalibrated %u", _inaAux.isCalibrated());
        // logDebugP("getCurrentLSB %.4f", _inaAux.getCurrentLSB());
        // logDebugP("getShunt %.4f", _inaAux.getShunt());
        // logDebugP("getMaxCurrent %.4f", _inaAux.getMaxCurrent());
#endif
        logIndentDown();
    }
    else
        logDebugP("AUX INA226 not found at address %u", OPENKNX_BPS_CURRENT_AUX_INA_ADDR);
}

float BusPowerSupplyModule::estimateBusLoad()
{
    TpUartDataLinkLayer* dll = knx.bau().getDataLinkLayer();
    TPUart::Statistics& statistics = dll->getTPUart().getStatistics();
    uint32_t currentTime = millis();
    uint32_t timeDiff = currentTime - _rxLastBusLoadTime;
    uint32_t currentBytes = statistics.getRxBusBytes();
    uint32_t bytesPerSecond = (currentBytes - _rxLastBusBytes) / timeDiff * 1000;
    _rxLastBusLoadTime = currentTime;
    _rxLastBusBytes = currentBytes;

    return (float)bytesPerSecond / (float)BUS_LOAD_MAX_BYTES_PER_SECOND;
}

void BusPowerSupplyModule::loop(bool configured)
{
    if (_firstLoop)
    {
        if (_pwrReadFromFlash)
        {
            if (_pwrActive == 1)
            {
                openknx.gpio.pinMode(OPENKNX_BPS_PWR2_SWITCH_ON_PIN, OUTPUT, true, !OPENKNX_BPS_PWR_SWITCH_ACTIVE_ON);
                openknx.gpio.pinMode(OPENKNX_BPS_PWR1_SWITCH_ON_PIN, OUTPUT, true, OPENKNX_BPS_PWR_SWITCH_ACTIVE_ON);
                logDebugP("PWR1 activated after restart");
            }
            else if (_pwrActive == 2)
            {
                openknx.gpio.pinMode(OPENKNX_BPS_PWR1_SWITCH_ON_PIN, OUTPUT, true, !OPENKNX_BPS_PWR_SWITCH_ACTIVE_ON);
                openknx.gpio.pinMode(OPENKNX_BPS_PWR2_SWITCH_ON_PIN, OUTPUT, true, OPENKNX_BPS_PWR_SWITCH_ACTIVE_ON);
                logDebugP("PWR2 activated after restart");
            }
        }
        else
        {
            openknx.gpio.pinMode(OPENKNX_BPS_PWR1_SWITCH_ON_PIN, OUTPUT, true, !OPENKNX_BPS_PWR_SWITCH_ACTIVE_ON);
            openknx.gpio.pinMode(OPENKNX_BPS_PWR2_SWITCH_ON_PIN, OUTPUT, true, !OPENKNX_BPS_PWR_SWITCH_ACTIVE_ON);
            logDebugP("No power supply activated after restart");
        }
        
        // no longer hold power supply
        openknx.gpio.pinMode(OPENKNX_BPS_PWR1_HOLD_PIN, OUTPUT, true, !OPENKNX_BPS_PWR_HOLD_ACTIVE_ON);
        openknx.gpio.pinMode(OPENKNX_BPS_PWR2_HOLD_PIN, OUTPUT, true, !OPENKNX_BPS_PWR_HOLD_ACTIVE_ON);

        // reset power supply hold after boot
        openknx.gpio.pinMode(OPENKNX_BPS_PWR1_RESET_PIN, OUTPUT, true, OPENKNX_BPS_PWR_RESET_ACTIVE_ON);
        openknx.gpio.pinMode(OPENKNX_BPS_PWR2_RESET_PIN, OUTPUT, true, OPENKNX_BPS_PWR_RESET_ACTIVE_ON);

        _firstLoop = false;
        _holdResetDelay = delayTimerInit();
    }

    if (_holdResetDelay > 0)
    {
        if (delayCheck(_holdResetDelay, HOLD_RESET_DELAY_MS))
        {
            openknx.gpio.pinMode(OPENKNX_BPS_PWR1_RESET_PIN, OUTPUT, true, !OPENKNX_BPS_PWR_RESET_ACTIVE_ON);
            openknx.gpio.pinMode(OPENKNX_BPS_PWR2_RESET_PIN, OUTPUT, true, !OPENKNX_BPS_PWR_RESET_ACTIVE_ON);

            logDebugP("Hold reset after boot delay");
            _holdResetDelay = 0;
        }
        else
            return;
    }

    float pwr1Voltage = (float)analogRead(OPENKNX_BPS_PWR1_CHECK_PIN) / (float)4095 * (float)3.3 * (float)OPENKNX_BPS_PWR_CHECK_FACTOR;
    bool pwr1Ok = pwr1Voltage > POWER_OK_THRESHOLD_VOLTAGE;
    if (_pwr1Ok != pwr1Ok)
    {
        _pwr1Ok = pwr1Ok;
        openknx.gpio.digitalWrite(OPENKNX_BPS_STATUS_PW1, _pwr1Ok ? OPENKNX_BPS_STATUS_ACTIVE_ON : !OPENKNX_BPS_STATUS_ACTIVE_ON);

        if (configured && ParamBPS_PowerSupply1ChangeSend)
            KoBPS_PowerSupply1Status.value(pwr1Ok, DPT_Switch);
    }

    float pwr2Voltage = (float)analogRead(OPENKNX_BPS_PWR2_CHECK_PIN) / (float)4095 * (float)3.3 * (float)OPENKNX_BPS_PWR_CHECK_FACTOR;
    bool pwr2Ok = pwr2Voltage > POWER_OK_THRESHOLD_VOLTAGE;
    if (_pwr2Ok != pwr2Ok)
    {
        _pwr2Ok = pwr2Ok;
        openknx.gpio.digitalWrite(OPENKNX_BPS_STATUS_PW2, _pwr2Ok ? OPENKNX_BPS_STATUS_ACTIVE_ON : !OPENKNX_BPS_STATUS_ACTIVE_ON);

        if (configured && ParamBPS_PowerSupply2ChangeSend)
            KoBPS_PowerSupply2Status.value(pwr2Ok, DPT_Switch);
    }

    uint8_t resetTime = configured ? ParamBPS_ResetTime : 10;
    if (_resetActive && delayCheck(_resetStarted, resetTime * 1000))
    {
        _resetActive = false;
        _resetStarted = 0;
        openknx.gpio.digitalWrite(OPENKNX_BPS_STATUS_RST, !OPENKNX_BPS_STATUS_ACTIVE_ON);

        logInfoP("Bus reset finished");
    }

    if (!_resetActive)
    {
        if (_recentPwrSupplySwitches > MAX_CURRENT_SWITCH_PER_SECOND &&
            (_pwrActive == 1 && !_pwr1Ok || _pwrActive == 2 && !_pwr2Ok))
        {
            pwr1Off();
            pwr2Off();
            _pwrActive = 0;
            _overcurrent = true;
            _overcurrentStarted = delayTimerInit();

            logErrorP("Too many power supply switches, all power off");
        }
        else if (_pwrActive == 1 && !_pwr1Ok)
        {
            if (_pwr2Ok)
            {
                pwr1Off();
                pwr2On();
                _pwrActive = 2;
                _pwrErrorLogged = false;

                _recentPwrSupplySwitches++;
                _lastPwrSupplySwitch = delayTimerInit();

                logInfoP("PWR1 failed, switched to PWR2");
            }
            else
            {
                if (!_pwrErrorLogged)
                {
                    _pwrErrorLogged = true;
                    logErrorP("PWR1 failed, PWR2 is NOT available, too!");

                    openknx.common.triggerSavePin();
                    logInfoP("SAVE triggered.");
                }
            }
        }
        else if (_pwrActive == 2 && !_pwr2Ok)
        {
            if (_pwr1Ok)
            {
                pwr2Off();
                pwr1On();
                _pwrActive = 1;
                _pwrErrorLogged = false;

                _recentPwrSupplySwitches++;
                _lastPwrSupplySwitch = delayTimerInit();

                logInfoP("PWR2 failed, switched to PWR1");
            }
            else
            {
                if (!_pwrErrorLogged)
                {
                    _pwrErrorLogged = true;
                    logErrorP("PWR2 failed, PWR1 is NOT available, too!");

                    openknx.common.triggerSavePin();
                    logInfoP("SAVE triggered.");
                }
            }
        }
        else if (_pwrActive == 0)
        {
            uint32_t retryDelay = 60000;
            if (_overcurrentRetryCount <= 1)
                retryDelay = 3000;
            else if (_overcurrentRetryCount == 2)
                retryDelay = 5000;
            else if (_overcurrentRetryCount == 3)
                retryDelay = 10000;
            else if (_overcurrentRetryCount == 4)
                retryDelay = 30000;

            if (!_overcurrent || delayCheck(_overcurrentStarted, retryDelay))
            {
                if (_pwr1Ok)
                {
                    pwr2Off();
                    pwr1On();
                    _pwrActive = 1;
                    _overcurrent = false;
                    _overcurrentRetryCount = 0;
                    _pwrErrorLogged = false;
                    logInfoP("Power supply started, PWR1 available, switching to PWR1");
                }
                else if (_pwr2Ok)
                {
                    pwr1Off();
                    pwr2On();
                    _pwrActive = 2;
                    _overcurrent = false;
                    _overcurrentRetryCount = 0;
                    _pwrErrorLogged = false;
                    logInfoP("Power supply started, PWR2 available, switching to PWR2");
                }
                else
                {
                    if (!_pwrErrorLogged)
                    {
                        _pwrErrorLogged = true;
                        logErrorP("Power supply started, no power supply available!");
                    }
                }
            }
        }
    }

    float busVoltage = _inaKnx.getBusVoltage();
    float busCurrent = _inaKnx.getCurrent();
    float auxVoltage = _inaAux.getBusVoltage();
    float auxCurrent = _inaAux.getCurrent();
    float busLoad = estimateBusLoad();
    float temperature = _temperature.getTemperature();

#ifdef OPENKNX_DEBUG
    if (delayCheck(_debugTimer, 1000))
    {
        logDebugP("PWR1 Voltage: %.2f V (%s), PWR2 Voltage: %.2f V (%s), active: %d", pwr1Voltage, _pwr1Ok ? "OK" : "NOT OK", pwr2Voltage, _pwr2Ok ? "OK" : "NOT OK", _pwrActive);
        logDebugP("KNX Power: %.2f mA at %.2f V", busCurrent * 1000, busVoltage);
        logDebugP("AUX Power: %.2f mA at %.2f V", auxCurrent * 1000, auxVoltage);
        logDebugP("Bus Load: %.2f %%, Temperature: %.2f °C", busLoad * 100, temperature);
        
        _debugTimer = delayTimerInit();
    }
#endif

    bool busOk = busVoltage > POWER_OK_THRESHOLD_VOLTAGE;
    if (_busOk != busOk)
    {
        _busOk = busOk;
        openknx.gpio.digitalWrite(OPENKNX_BPS_STATUS_BUS, _busOk ? !OPENKNX_BPS_STATUS_ACTIVE_ON : OPENKNX_BPS_STATUS_ACTIVE_ON);
    }

    float totalCurrentMa = (busCurrent + auxCurrent) * 1000;
    bool currentOk = _resetActive || totalCurrentMa < CURRENT_MAX_THRESHOLD_MA;
    if (_currentOk != currentOk)
    {
        _currentOk = currentOk;
        openknx.gpio.digitalWrite(OPENKNX_BPS_STATUS_MAX, _currentOk ? !OPENKNX_BPS_STATUS_ACTIVE_ON : OPENKNX_BPS_STATUS_ACTIVE_ON);
    }

    bool maxOverload = totalCurrentMa > CURRENT_OVERLOAD_THRESHOLD_MAX_MA;
    bool shortOverload = totalCurrentMa > CURRENT_OVERLOAD_THRESHOLD_SHORT_MA;

    if (_resetActive || !shortOverload)
        _overcurrentShortStarted = 0;

    if (!_resetActive)
    {
        if (maxOverload)
        {
            pwr1Off();
            pwr2Off();
            _pwrActive = 0;
            _overcurrent = true;
            _overcurrentStarted = delayTimerInit();
            _overcurrentShortStarted = 0;
            if (_overcurrentRetryCount < 255)
                _overcurrentRetryCount++;

            logErrorP("Maximum overload exceeded (%.2f mA > %u mA), all power off", totalCurrentMa, CURRENT_OVERLOAD_THRESHOLD_MAX_MA);
        }
        else if (shortOverload)
        {
            if (_overcurrentShortStarted == 0)
                _overcurrentShortStarted = delayTimerInit();

            if (delayCheck(_overcurrentShortStarted, CURRENT_OVERLOAD_THRESHOLD_SHORT_TIME_MS))
            {
                pwr1Off();
                pwr2Off();
                _pwrActive = 0;
                _overcurrent = true;
                _overcurrentStarted = delayTimerInit();
                _overcurrentShortStarted = 0;
                if (_overcurrentRetryCount < 255)
                    _overcurrentRetryCount++;

                logErrorP("Short-time overload exceeded for too long (%.2f mA > %u mA for %u ms), all power off", totalCurrentMa, CURRENT_OVERLOAD_THRESHOLD_SHORT_MA, CURRENT_OVERLOAD_THRESHOLD_SHORT_TIME_MS);
            }
        }
    }

    if (_lastPwrSupplySwitch > 0 && delayCheck(_lastPwrSupplySwitch, POWER_SUPPLY_SWITCH_RECENT_RESET_MS))
    {
        _lastPwrSupplySwitch = 0;
        _recentPwrSupplySwitches = 0;
    }

    bool resetPressed = openknx.gpio.digitalRead(OPENKNX_BPS_SWITCH_RST) == OPENKNX_BPS_SWITCH_ACTIVE_ON;
    if (resetPressed && _resetStarted == 0)
    {
        _resetActive = true;
        _resetStarted = delayTimerInit();
        openknx.gpio.digitalWrite(OPENKNX_BPS_STATUS_RST, OPENKNX_BPS_STATUS_ACTIVE_ON);
        logInfoP("Bus reset started, all power off for %d sec.", resetTime);

        pwr1Off();
        pwr2Off();
        _pwrActive = 0;
    }

    if (configured)
    {
        if (ParamBPS_PowerSupply1SendCyclicTimeMS > 0 && delayCheck(_powerSupply1SendTimer, ParamBPS_PowerSupply1SendCyclicTimeMS))
        {
            KoBPS_PowerSupply1Status.value(_pwr1Ok, DPT_Switch);
            _powerSupply1SendTimer = delayTimerInit();
        }

        if (ParamBPS_PowerSupply2SendCyclicTimeMS > 0 && delayCheck(_powerSupply2SendTimer, ParamBPS_PowerSupply2SendCyclicTimeMS))
        {
            KoBPS_PowerSupply2Status.value(_pwr2Ok, DPT_Switch);
            _powerSupply2SendTimer = delayTimerInit();
        }

        processSendValue(KoBPS_BusVoltage, DPT_Value_Electric_Potential, ParamBPS_BusVoltageChangeSend, ParamBPS_BusVoltageSendMinChangePercent, ParamBPS_BusVoltageSendMinChangeAbsolute, ParamBPS_BusVoltageSendCyclicTimeMS, _busVoltageSendTimer, _lastBusVoltageSent, busVoltage);
        processSendValue(KoBPS_BusCurrent, DPT_Value_Electric_Current, ParamBPS_BusCurrentChangeSend, ParamBPS_BusCurrentSendMinChangePercent, ParamBPS_BusCurrentSendMinChangeAbsolute, ParamBPS_BusCurrentSendCyclicTimeMS, _busCurrentSendTimer, _lastBusCurrentSent, busCurrent, 1000);
        processSendValue(KoBPS_BusLoad, DPT_Scaling, ParamBPS_BusLoadChangeSend, ParamBPS_BusLoadSendMinChangePercent, ParamBPS_BusLoadSendMinChangeAbsolute, ParamBPS_BusLoadSendCyclicTimeMS, _busLoadSendTimer, _lastBusLoadSent, busLoad);

        processSendValue(KoBPS_AuxVoltage, DPT_Value_Electric_Potential, ParamBPS_AuxVoltageChangeSend, ParamBPS_AuxVoltageSendMinChangePercent, ParamBPS_AuxVoltageSendMinChangeAbsolute, ParamBPS_AuxVoltageSendCyclicTimeMS, _auxVoltageSendTimer, _lastAuxVoltageSent, auxVoltage);
        processSendValue(KoBPS_AuxCurrent, DPT_Value_Electric_Current, ParamBPS_AuxCurrentChangeSend, ParamBPS_AuxCurrentSendMinChangePercent, ParamBPS_AuxCurrentSendMinChangeAbsolute, ParamBPS_AuxCurrentSendCyclicTimeMS, _auxCurrentSendTimer, _lastAuxCurrentSent, auxCurrent, 1000);
        
        processSendValue(KoBPS_Temperature, DPT_Value_Temp, ParamBPS_TemperatureChangeSend, ParamBPS_TemperatureSendMinChangePercent, ParamBPS_TemperatureSendMinChangeAbsolute, ParamBPS_TemperatureSendCyclicTimeMS, _temperatureSendTimer, _lastTemperatureSent, temperature);
    }
}

void BusPowerSupplyModule::processSendValue(GroupObject& ko, Dpt dpt, bool send, uint8_t sendMinChangePercent, uint16_t sendMinChangeAbsolute, uint32_t sendCyclicTimeMS, uint32_t& cyclicSendTimer, float& lastSentValue, float currentValue, uint16_t checkMultiply)
{
    if (!send)
        return;

    uint16_t currentDifference = round(abs(lastSentValue - currentValue * checkMultiply));
    if (currentDifference > 0)
    {
        if ((lastSentValue == 0 || currentDifference >= lastSentValue * sendMinChangePercent / checkMultiply) &&
            currentDifference >= sendMinChangeAbsolute)
        {
            ko.value(currentValue, dpt);
            lastSentValue = currentValue * checkMultiply;
        }
        else
            ko.valueNoSend(currentValue, dpt);
    }

    if (sendCyclicTimeMS > 0 && delayCheckMillis(cyclicSendTimer, sendCyclicTimeMS))
    {
        ko.value(currentValue, dpt);
        lastSentValue = currentValue * checkMultiply;
        cyclicSendTimer = delayTimerInit();
    }
}

void BusPowerSupplyModule::pwr1On()
{
    logDebugP("Turn PWR1 on");
    openknx.gpio.digitalWrite(OPENKNX_BPS_PWR1_SWITCH_ON_PIN, OPENKNX_BPS_PWR_SWITCH_ACTIVE_ON);
}

void BusPowerSupplyModule::pwr1Off()
{
    logDebugP("Turn PWR1 off");
    openknx.gpio.digitalWrite(OPENKNX_BPS_PWR1_SWITCH_ON_PIN, !OPENKNX_BPS_PWR_SWITCH_ACTIVE_ON);
}

void BusPowerSupplyModule::pwr2On()
{
    logDebugP("Turn PWR2 on");
    openknx.gpio.digitalWrite(OPENKNX_BPS_PWR2_SWITCH_ON_PIN, OPENKNX_BPS_PWR_SWITCH_ACTIVE_ON);
}

void BusPowerSupplyModule::pwr2Off()
{
    logDebugP("Turn PWR2 off");
    openknx.gpio.digitalWrite(OPENKNX_BPS_PWR2_SWITCH_ON_PIN, !OPENKNX_BPS_PWR_SWITCH_ACTIVE_ON);
}

void BusPowerSupplyModule::readFlash(const uint8_t *data, const uint16_t size)
{
    if (size == 0)
        return;

    logDebugP("Reading state from flash");
    logIndentUp();

    uint8_t version = openknx.flash.readByte();
    if (version != OPENKNX_BPS_FLASH_VERSION)
    {
        logDebugP("Invalid flash version %u", version);
        return;
    }

    uint32_t magicWord = openknx.flash.readInt();
    if (magicWord != OPENKNX_BPS_FLASH_MAGIC_WORD)
    {
        logDebugP("Flash content invalid");
        return;
    }

    _pwrActive = openknx.flash.readByte();
    if (_pwrActive == 1 || _pwrActive == 2)
    {
        logDebugP("Power supply active from flash: %u", _pwrActive);
        _pwrReadFromFlash = true;
    }
    else
    {
        _pwrActive = 0;
        logErrorP("Power supply active from flash invalid: %u", _pwrActive);
    }

    logIndentDown();
}

void BusPowerSupplyModule::writeFlash()
{
    openknx.flash.writeByte(OPENKNX_BPS_FLASH_VERSION);
    openknx.flash.writeInt(OPENKNX_BPS_FLASH_MAGIC_WORD);

    openknx.flash.writeByte(_pwrActive);

    logDebugP("State written to flash");
}

uint16_t BusPowerSupplyModule::flashSize()
{
    return 1 + 4 + 1; // version + magic word + lastPwrActive
}

void BusPowerSupplyModule::savePower()
{

}

bool BusPowerSupplyModule::restorePower()
{
    bool success = true;
    
    return success;
}

void BusPowerSupplyModule::processBeforeRestart()
{
    openknx.flash.save();

    if (_pwrActive == 1)
    {
        logDebugP("Restart and hold PWR1 active");
        openknx.gpio.digitalWrite(OPENKNX_BPS_PWR1_HOLD_PIN, OPENKNX_BPS_PWR_HOLD_ACTIVE_ON);
    }
    else if (_pwrActive == 2)
    {
        logDebugP("Restart and hold PWR2 active");
        openknx.gpio.digitalWrite(OPENKNX_BPS_PWR2_HOLD_PIN, OPENKNX_BPS_PWR_HOLD_ACTIVE_ON);
    }
}

void BusPowerSupplyModule::showHelp()
{
    //logInfo("sa run test mode", "Test all channels one after the other.");
}

bool BusPowerSupplyModule::processCommand(const std::string cmd, bool diagnoseKo)
{
    if (cmd.substr(0, 2) != "bs")
        return false;

    if (cmd.length() == 10 && cmd.substr(3, 7) == "pwr1 on")
    {
        pwr1On();
        return true;
    }
    else if (cmd.length() == 11 && cmd.substr(3, 8) == "pwr1 off")
    {
        pwr1Off();
        return true;
    }
    else if (cmd.length() == 10 && cmd.substr(3, 7) == "pwr2 on")
    {
        pwr2On();
        return true;
    }
    else if (cmd.length() == 11 && cmd.substr(3, 8) == "pwr2 off")
    {
        pwr2Off();
        return true;
    }

    // Commands starting with ba are our diagnose commands
    logInfoP("ba (BusPowerSupply) command with bad args");
    if (diagnoseKo)
    {
        openknx.console.writeDiagenoseKo("ba: bad args");
    }
    return true;
}

void BusPowerSupplyModule::runTestMode()
{
    // logInfoP("Starting test mode");
    // logIndentUp();


    // logInfoP("Testing finished.");
    // logIndentDown();
}
