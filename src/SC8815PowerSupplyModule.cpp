#include "SC8815PowerSupplyModule.h"
#include "ModuleVersionCheck.h"

BusPowerSupplyModule openknxBusPowerSupplyModule;

// 过流/短路等故障后的重试等待时间(ms)，超出后一直使用最后一个值
static const uint16_t cRetryDelays[] = {3000, 5000, 10000, 30000, 60000};
static const uint8_t cRetryDelayCount = sizeof(cRetryDelays) / sizeof(cRetryDelays[0]);

/**
 * @brief 简单的 I2C 存在性探测
 */
static bool i2cPing(TwoWire &wire, uint8_t address)
{
    wire.beginTransmission(address);
    return wire.endTransmission() == 0;
}

/**
 * @brief ETS 的开关频率参数(kHz) 映射到 SC8815 寄存器值
 */
static SCHWI_FREQ frequencyFromKHz(uint16_t kHz)
{
    switch (kHz)
    {
        case 150:
            return SCHWI_FREQ_150KHz;
        case 450:
            return SCHWI_FREQ_450KHz;
        case 300:
        default:
            return SCHWI_FREQ_300KHz_1;
    }
}

/**
 * @brief 一位小数格式化(避免在日志里使用 %f)
 */
static std::string oneDecimalString(float value)
{
    const int16_t tenths = (int16_t)(value * 10.0f);
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%d.%d", tenths / 10, abs(tenths % 10));
    return std::string(buffer);
}

BusPowerSupplyModule::BusPowerSupplyModule()
{
    // ---- 总线侧 ----
    _bus.label = "Bus";
    _bus.isAux = false;
    _bus.wire = &OPENKNX_GPIO_WIRE;
    _bus.chipAddress = SC8815_I2C_ADDR;
    _bus.tempAddress = OPENKNX_BPS_TMP_BUS_POWER_SUPPLY_ADDR;
    _bus.intPin = OPENKNX_BPS_SC8815_BUS_POWER_SUPPLY_INT_PIN;
    _bus.clrPin = OPENKNX_BPS_SC8815_BUS_POWER_SUPPLY_CLR_PIN;
    _bus.cePin = OPENKNX_BPS_SC8815_BUS_POWER_SUPPLY_CE_PIN;
    _bus.resetPin = OPENKNX_BPS_SC8815_BUS_POWER_SUPPLY_RESET_PIN;
    _bus.alertPin = OPENKNX_BPS_INA_BUS_POWER_SUPPLY_ALERT_PIN;
    _bus.inaAddress = OPENKNX_BPS_INA_BUS_POWER_SUPPLY_ADDR;
    _bus.inaShuntOhm = OPENKNX_BPS_INA_BUS_POWER_SUPPLY_SHUNT;
    _bus.clrActive = OPENKNX_BPS_SC8815_BUS_POWER_SUPPLY_CLR_PIN_ACTIVE_ON;
    _bus.ceActive = OPENKNX_BPS_SC8815_BUS_POWER_SUPPLY_CE_PIN_ACTIVE_ON;
    _bus.resetActive = OPENKNX_BPS_SC8815_BUS_POWER_SUPPLY_RESET_PIN_ACTIVE_ON;
    _bus.sc = &_scBus;
    _bus.ina = &_inaBus;
    _bus.tmp = &_tmpBus;

    // ---- 辅助侧 ----
    _aux.label = "Aux";
    _aux.isAux = true;
    _aux.wire = &OPENKNX_GPIO_WIRE1;
    _aux.chipAddress = SC8815_I2C_ADDR;
    _aux.tempAddress = OPENKNX_BPS_TMP_AUX_POWER_SUPPLY_ADDR;
    _aux.intPin = OPENKNX_BPS_SC8815_AUX_POWER_SUPPLY_INT_PIN;
    _aux.clrPin = OPENKNX_BPS_SC8815_AUX_POWER_SUPPLY_CLR_PIN;
    _aux.cePin = OPENKNX_BPS_SC8815_AUX_POWER_SUPPLY_CE_PIN;
    _aux.resetPin = OPENKNX_BPS_SC8815_AUX_POWER_SUPPLY_RESET_PIN;
    _aux.alertPin = OPENKNX_BPS_INA_AUX_POWER_SUPPLY_ALERT_PIN;
    _aux.inaAddress = OPENKNX_BPS_INA_AUX_POWER_SUPPLY_ADDR;
    _aux.inaShuntOhm = OPENKNX_BPS_INA_AUX_POWER_SUPPLY_SHUNT;
    _aux.clrActive = OPENKNX_BPS_SC8815_AUX_POWER_SUPPLY_CLR_PIN_ACTIVE_ON;
    _aux.ceActive = OPENKNX_BPS_SC8815_AUX_POWER_SUPPLY_CE_PIN_ACTIVE_ON;
    _aux.resetActive = OPENKNX_BPS_SC8815_AUX_POWER_SUPPLY_RESET_PIN_ACTIVE_ON;
    _aux.sc = &_scAux;
    _aux.ina = &_inaAux;
    _aux.tmp = &_tmpAux;
}

const std::string BusPowerSupplyModule::name()
{
    return "BusPower";
}

const std::string BusPowerSupplyModule::version()
{
    return MODULE_BusPowerSupply_Version;
}

void BusPowerSupplyModule::setup(bool configured)
{
    // 总线侧 I2C 由 OpenKNX GPIO Manager 初始化，辅助侧在这里初始化
    OPENKNX_GPIO_WIRE1.setSDA(OPENKNX_GPIO_SDA1);
    OPENKNX_GPIO_WIRE1.setSCL(OPENKNX_GPIO_SCL1);
    OPENKNX_GPIO_WIRE1.begin();
    OPENKNX_GPIO_WIRE1.setClock(OPENKNX_GPIO_CLOCK1);

    applyParameters(configured);

    initPins(_bus);
    initPins(_aux);

    // INA238：输出侧测量 + 过流报警（报警信号经触发器直接关断 SC8815）
    BpsUnit *units[2] = {&_bus, &_aux};
    for (BpsUnit *u : units)
    {
        if (!u->ina->begin())
        {
            u->faultCode |= BPS_FAULT_INA_NO_ANSWER;
            logErrorP("%s: INA238 not found at 0x%02X", u->label, u->inaAddress);
            continue;
        }

        u->ina->reset();
        u->ina->setADCRange(false); // ±163.84 mV -> 10 mΩ 上最大 16.384 A, 远高于任何限流
        u->ina->setMaxCurrentShunt(BPS_INA_MAX_CURRENT_A, u->inaShuntOhm);
        u->ina->setAverage(INA238_16_SAMPLES);
        u->ina->setMode(INA238_MODE_CONT_TEMP_BUS_SHUNT);

        // 过流门限 + 锁存报警：ALERT 低有效, 触发锁存器把 SC8815 的 PSTOP 拉高
        u->ina->setOverCurrentLimit((uint32_t)u->currentLimit_mA * BPS_OVERCURRENT_ALERT_PERCENT / 100);
        u->ina->setDiagnoseAlertBit(INA238_DIAG_SHUNT_OVER_LIMIT);
        u->ina->setDiagnoseAlertBit(INA238_DIAG_POWER_OVER_LIMIT);
        u->ina->setDiagnoseAlertBit(INA238_DIAG_ALERT_LATCH);

        logDebugP("%s: INA238 ready (shunt %u mOhm, range %u mA, alert at %u mA)", u->label,
                  (uint32_t)(u->inaShuntOhm * 1000.0f), (uint32_t)BPS_INA_MAX_CURRENT_A * 1000,
                  (uint32_t)u->currentLimit_mA * BPS_OVERCURRENT_ALERT_PERCENT / 100);
    }

    // TMP102 温度传感器
    for (BpsUnit *u : units)
    {
        if (!u->tmp->begin(u->tempAddress, *u->wire))
        {
            u->faultCode |= BPS_FAULT_TEMP_NO_ANSWER;
            logWarningP("%s: temperature sensor not found at 0x%02X (using INA238 temperature)", u->label, u->tempAddress);
        }
        else
        {
            u->tmp->setExtendedMode(true); // -55..+150 °C
            u->tmp->setConversionRate(2);  // 4 Hz
        }
    }

    // 总线电源必须供电（否则设备无法被编程/总线没有电压）
    if (!startUnit(_bus))
        logErrorP("Bus: output not available, retry will follow");

    if (_aux.enabled)
    {
        if (!startUnit(_aux))
            logErrorP("Aux: output not available, retry will follow");
    }
    else
    {
        powerDownUnit(_aux);
        logInfoP("Aux: disabled by parameter");
    }
}

void BusPowerSupplyModule::loop(bool configured)
{
    const uint32_t now = delayTimerInit();

    updateUnit(_bus, now);
    updateUnit(_aux, now);

    if (configured)
    {
        sendUnitValues(_bus);
        if (_aux.enabled)
            sendUnitValues(_aux);
    }

    if (_testMode && delayCheck(_testTimer, 1000))
    {
        _testTimer = delayTimerInit();
        logUnit(_bus);
        logUnit(_aux);
    }
}

void BusPowerSupplyModule::processInputKo(GroupObject &ko)
{
    const uint16_t asap = ko.asap();

    if (asap == BPS_KoBusReset || asap == BPS_KoAuxReset)
    {
        if ((bool)ko.value(DPT_Switch))
        {
            BpsUnit &u = (asap == BPS_KoBusReset) ? _bus : _aux;
            startReset(u, resetDurationMS(u));
            openknx.console.writeDiagenoseKo("%s: reset triggered by group object", u.label);
        }
        return;
    }
    logDebugP("processInputKo asap %u", asap);
}

/* ===========================================================================
 *  硬件与控制
 * ======================================================================== */

void BusPowerSupplyModule::initPins(BpsUnit &u)
{
    // SC8815 INT 与 INA238 ALERT 都是开漏输出，因此使能内部上拉。
    // ALERT 同时直接触发外部触发器把 SC8815 的 PSTOP 拉高（见头文件），
    // 该节点由 INA238 驱动，固件只能读，绝不能写。
    openknx.gpio.pinMode(u.intPin, INPUT_PULLUP);
    openknx.gpio.pinMode(u.alertPin, INPUT_PULLUP);

    // 外部复位按键（上拉输入, 低 = 按下, 需长按 3 s 以上）与锁存器 CLR / 芯片使能引脚
    openknx.gpio.pinMode(u.resetPin, INPUT_PULLUP);
    openknx.gpio.pinMode(u.clrPin, OUTPUT, true, !u.clrActive);
    openknx.gpio.pinMode(u.cePin, OUTPUT, true, !u.ceActive);
}

/**
 * @brief 接通/断开输出通路
 *
 * 输出通断由 SC8815 的 PGATE 引脚完成（驱动外部 PMOS 栅极），
 * 不改变芯片配置，因此复位结束后可直接恢复输出。
 */
void BusPowerSupplyModule::setOutputPath(BpsUnit &u, bool enable)
{
    if (u.chipOk)
        u.sc->setPgateEnable(enable);

    u.outputOn = enable;
    if (!enable)
        u.statusOk = false;
}

/**
 * @brief 读取外部复位按键
 */
bool BusPowerSupplyModule::readButton(BpsUnit &u)
{
    return openknx.gpio.digitalRead(u.resetPin) == u.resetActive;
}

/**
 * @brief 按键处理：去抖 + 长按判定
 *
 * 复位按键必须连续按住 BPS_BUTTON_HOLD_MS（3 s）以上才触发该路复位；
 * 短按（含误碰）只记一条日志，不动输出——总线电源是给 KNX 供电的，
 * 不能一碰就断。
 * 触发后若继续按住，updateUnit() 会把复位顺延（相当于一直按着前面板按键）。
 */
void BusPowerSupplyModule::updateButton(BpsUnit &u, uint32_t now)
{
    const bool raw = readButton(u);

    // ---- 原始电平有变化 -> 重新去抖计时，本次不判断 ----
    if (raw != u.buttonLast)
    {
        u.buttonLast = raw;
        u.buttonTimer = now;
        return;
    }

    // ---- 去抖未完成，保持现状 ----
    if (u.buttonPressed != u.buttonLast && !delayCheckMillis(u.buttonTimer, BPS_BUTTON_DEBOUNCE_MS))
        return;

    const bool wasPressed = u.buttonPressed;
    u.buttonPressed = u.buttonLast;

    if (wasPressed != u.buttonPressed)
    {
        if (u.buttonPressed)
        {
            u.buttonHoldTimer = now; // 刚按下：开始长按计时
            logDebugP("%s: reset button pressed, hold %u ms to trigger", u.label, BPS_BUTTON_HOLD_MS);
        }
        else
        {
            if (!u.buttonHoldTriggered)
                logInfoP("%s: reset button released after %u ms, too short (needs %u ms), ignored", u.label,
                         (uint32_t)(now - u.buttonHoldTimer), BPS_BUTTON_HOLD_MS);

            u.buttonHoldTriggered = false; // 松开：长按标志清零，下次要重新按足
        }
    }

    // ---- 长按满 BPS_BUTTON_HOLD_MS：只触发一次 ----
    if (u.buttonPressed && !u.buttonHoldTriggered && delayCheckMillis(u.buttonHoldTimer, BPS_BUTTON_HOLD_MS))
    {
        u.buttonHoldTriggered = true;
        logInfoP("%s: reset button held %u ms, triggering reset", u.label, BPS_BUTTON_HOLD_MS);
        startReset(u, resetDurationMS(u), true);
    }
}

bool BusPowerSupplyModule::configureChip(BpsUnit &u)
{
    if (!u.sc->begin(BPS_SHUNT_MOHM, BPS_SHUNT_MOHM, BPS_FB_RUP_KOHM, BPS_FB_RDOWN_KOHM))
    {
        logErrorP("%s: SC8815 not answering at 0x%02X", u.label, u.chipAddress);
        return false;
    }

    // VBAT 侧（输入侧电源轨）配置
    BatteryConfig battery = {};
    battery.IRCOMP = SCBAT_IRCOMP_0mR;
    battery.VBAT_SEL = BPS_VBAT_SEL;
    battery.CSEL = BPS_VBAT_CSEL;
    battery.VCELL = BPS_VBAT_VCELL;
    u.sc->configBattery(battery);

    // 注: VBATS 外部分压决定的 VBAT 过压点不在里算 —— 芯片要 ADC 跑起来之后
    // 才能读到 VBATS 引脚电压, 所以放到 sampleUnit() 里实测反推, 见 checkVbatOverVoltage()

    // 硬件运行参数：放电(OTG)模式, 外部 FB, ADC 打开
    HardwareConfig hardware = {};
    hardware.IBAT_RATIO = SCHWI_IBAT_RATIO_12x;
    hardware.IBUS_RATIO = SCHWI_IBUS_RATIO_3x;
    hardware.VBAT_RATIO = SCHWI_VBAT_RATIO_12_5x;
    hardware.VBUS_RATIO = SCHWI_VBUS_RATIO_12_5x;
    hardware.VINREG_Ratio = SCHWI_VINREG_RATIO_100x;
    hardware.SW_FREQ = frequencyFromKHz(u.frequencyKHz);
    hardware.DeadTime = SCHWI_DT_40ns;
    hardware.ICHAR = SCHWI_ICHAR_IBUS;
    hardware.TRICKLE = SCHWI_TRICKLE_Disable;
    hardware.TERM = SCHWI_TERM_Disable;
    hardware.FB_Mode = BPS_VBUS_FB_MODE;
    hardware.TRICKLE_SET = SCHWI_TRICKLE_SET_70;
    hardware.OVP = SCHWI_OVP_Enable;
    hardware.DITHER = SCHWI_DITHER_Disable;
    hardware.SLEW_SET = SCHWI_SLEW_1mV_us;
    hardware.ADC_SCAN = SCHWI_ADC_Enable;
    hardware.ILIM_BW = SCHWI_ILIM_BW_5KHz;
    hardware.LOOP = SCHWI_LOOP_Normal;
    hardware.ShortFoldBack = SCHWI_SFB_Disable; // 带载启动期间先关闭折返, 软启动后再打开
    hardware.EOC = SCHWI_EOC_1_25;
    hardware.PFM = SCHWI_PFM_Enable;
    u.sc->configHardware(hardware);

    // 输出目标值与限流
    u.sc->setOutputVoltage(u.targetVoltage_mV);
    u.sc->setBusCurrentLimit(u.currentLimit_mA);

    // 输入侧(IBAT)限流：输入电流 = 输出功率 / 输入电压，必须按最低输入电压
    // (BPS_INPUT_VOLTAGE_MIN_MV = 18V) 计算，否则在 18V 输入时会被输入限流
    // 压住输出。例：29V / 1280mA 输出、18V 输入 -> 约 2.1A 输入电流。
    //      输入限流 = 输出功率 / 最低输入电压 × 1.15（留效率损失与余量）
    uint32_t batteryLimit =
        (uint32_t)((float)u.currentLimit_mA * u.targetVoltage_mV * 1.15f / BPS_INPUT_VOLTAGE_MIN_MV);
    if (batteryLimit > BPS_INPUT_CURRENT_LIMIT_MAX_MA)
        batteryLimit = BPS_INPUT_CURRENT_LIMIT_MAX_MA;
    u.sc->setBatteryCurrentLimit((uint16_t)batteryLimit);

    // 进入放电(OTG)模式；输出 PMOS 先保持断开, 由 startUnit() 释放
    u.sc->setOtgEnable(true);
    u.sc->setPgateEnable(false);

    // 回读校验：DAC 只有 10 bit, 目标值与实际写入值之间会有一个步进的量化误差
    // （步进 = 2mV x 外部分压比, 200k/13.3k 时为 32 mV），因此把实际值也打印出来。
    const uint16_t maxOutput = u.sc->getMaxOutputVoltage();
    logInfoP("%s: SC8815 configured (%u mV target -> DAC %u mV, step %u mV, max %u mV, limit %u mA, %u kHz)", u.label,
             u.targetVoltage_mV, u.sc->getOutputVoltage(), u.sc->getOutputVoltageStep(), maxOutput, u.currentLimit_mA,
             u.frequencyKHz);
    if (maxOutput < u.targetVoltage_mV)
        logWarningP("%s: target voltage %u mV above FB limit %u mV - check BPS_FB_RUP_KOHM/BPS_FB_RDOWN_KOHM", u.label,
                    u.targetVoltage_mV, maxOutput);

    return true;
}

/**
 * @brief 用 ALERT 引脚把 SC8815 的 PSTOP 拉高 / 释放（仿开漏）
 *
 * ALERT 这根线同时接 INA238 的 ALERT 输出（开漏）和 SN74LVC1G74 的 PRE，
 * 触发器的 Q 接 SC8815 的 PSTOP。因此：
 *   拉低 -> PRE 有效 -> Q 高 -> PSTOP 高（功率级强制停，并锁存）
 *   释放 -> PRE 无效；PSTOP 是否回低由 CLR 决定（见 clearLatch()）
 * 只能“拉低 / 释放”，不能推挽驱高：INA238 是开漏输出，驱高会短路。
 * 固件主动拉低期间 pstopForced = true，过流检测不会把这次拉低当成报警。
 * 手册要求标 [PSTOP] 的寄存器必须在 PSTOP 为高时写入，所以 startUnit() 先用它。
 */
void BusPowerSupplyModule::setPstop(BpsUnit &u, bool high)
{
    u.pstopForced = high;

    if (high)
        openknx.gpio.pinMode(u.alertPin, OUTPUT, true, LOW); // 开漏拉低 -> PSTOP 高
    else
        openknx.gpio.pinMode(u.alertPin, INPUT_PULLUP); // 释放，交回上拉 / INA238
}

void BusPowerSupplyModule::clearLatch(BpsUnit &u)
{
    // 0) 先释放固件对 ALERT 的拉低：只要 PRE 还被按住, CLR 也拉不低 PSTOP
    setPstop(u, false);

    // 1) INA238 报警标志：该位同时是报警使能位（写 1 清除），因此先清位再重新置位
    u.ina->clearDiagnoseAlertBit(INA238_DIAG_SHUNT_OVER_LIMIT);
    u.ina->clearDiagnoseAlertBit(INA238_DIAG_POWER_OVER_LIMIT);
    u.ina->setDiagnoseAlertBit(INA238_DIAG_SHUNT_OVER_LIMIT);
    u.ina->setDiagnoseAlertBit(INA238_DIAG_POWER_OVER_LIMIT);
    u.ina->setDiagnoseAlertBit(INA238_DIAG_ALERT_LATCH);

    // 2) SN74LVC1G74 复位：Q = 0 -> PSTOP 低 -> 允许功率输出
    openknx.gpio.digitalWrite(u.clrPin, u.clrActive);
    delayMicroseconds(BPS_LATCH_CLEAR_US);
    openknx.gpio.digitalWrite(u.clrPin, !u.clrActive);
}

bool BusPowerSupplyModule::startUnit(BpsUnit &u)
{
    // 先软关断：PGATE 断开外部 PMOS + 停止开关
    disableOutput(u);

    // 芯片使能（CE 低有效），等内部 LDO 稳定
    openknx.gpio.digitalWrite(u.cePin, u.ceActive);
    delay(BPS_CHIP_STARTUP_MS);

    // 配置前先把 PSTOP 拉高（拉低 ALERT）：手册要求标 [PSTOP] 的寄存器
    // （VBAT_SET / RATIO / CTRL*）必须在 PSTOP 为高时写入。
    setPstop(u, true);

    if (!configureChip(u))
    {
        setPstop(u, false);
        u.chipOk = false;
        u.outputOn = false;
        return false;
    }

    u.chipOk = true;

    // 配置完成：清锁存（释放 ALERT + 清 INA238 标志 + CLR 脉冲）-> PSTOP 回低
    clearLatch(u);

    // 核对 ALERT：清完还是低 => INA238 仍在报警（负载还过流），
    // 就算打开 PGATE 也不会有输出，直接判失败交给重试逻辑退避重来。
    if (!openknx.gpio.digitalRead(u.alertPin))
    {
        logWarningP("%s: over current latch not cleared (ALERT still low), output stays off", u.label);
        u.outputOn = false;
        return false;
    }

    // 接通输出通路（PGATE 驱动外部 PMOS）
    setOutputPath(u, true);
    u.sfbPending = true;
    u.sfbTimer = delayTimerInit();

    logInfoP("%s: output on (%u mV / %u mA / %u kHz)", u.label, u.targetVoltage_mV, u.currentLimit_mA, u.frequencyKHz);
    return true;
}

void BusPowerSupplyModule::disableOutput(BpsUnit &u)
{
    // 断开输出通路（PGATE 关断外部 PMOS），并停止开关变换
    setOutputPath(u, false);
    if (u.chipOk)
        u.sc->setOtgEnable(false);

    u.sfbPending = false;
}

void BusPowerSupplyModule::powerDownUnit(BpsUnit &u)
{
    disableOutput(u);

    openknx.gpio.digitalWrite(u.clrPin, !u.clrActive);
    openknx.gpio.digitalWrite(u.cePin, !u.ceActive);

    u.resetByButton = false;
    u.buttonPressed = false;
    u.outputVoltage_mV = 0.0f;
    u.outputCurrent_mA = 0.0f;
    u.outputLoadPercent = 0.0f;
    u.statusOk = false;
}

void BusPowerSupplyModule::startReset(BpsUnit &u, uint32_t durationMS, bool fromButton)
{
    if (!u.enabled)
    {
        logInfoP("%s: reset ignored, output disabled", u.label);
        return;
    }

    // 只断开输出通路（PGATE），芯片保持工作，复位结束后可直接恢复输出
    setOutputPath(u, false);

    u.resetTimer = delayTimerInit();
    u.resetDurationMS = durationMS;
    u.resetByButton = fromButton;
    u.retryCount = 0;
    u.retryTimer = 0;

    logInfoP("%s: reset started (%s), output off for %u ms", u.label, fromButton ? "button" : "group object", durationMS);
}

/* ===========================================================================
 *  运行
 * ======================================================================== */

void BusPowerSupplyModule::applyParameters(bool configured)
{
    // 总线电源：输出电压固定（ETS 侧无参数），限流与开关频率可参数化
    _bus.enabled = true;
    _bus.targetVoltage_mV = BPS_BUS_TARGET_VOLTAGE_MV;
    _bus.currentLimit_mA = configured ? ParamBPS_SC8815BusCurrent : 640;
    _bus.frequencyKHz = configured ? ParamBPS_SC8815Frequency : 300;

    // 复位时间：share.xml 只有一个“复位时间”参数, 总线与辅助共用
    _resetTimeSeconds = configured ? (uint8_t)ParamBPS_ResetTime : 10;
    if (_resetTimeSeconds == 0)
        _resetTimeSeconds = 1;

    // 辅助电源：使能/电压/限流/频率均可参数化
    _aux.enabled = configured ? ParamBPS_SC8815AuxEnabled : false;
    _aux.targetVoltage_mV = (uint16_t)(configured ? ParamBPS_SC8815AuxVoltage : 24) * 1000;
    _aux.currentLimit_mA = configured ? ParamBPS_SC8815AuxCurrent : 960;
    _aux.frequencyKHz = configured ? ParamBPS_AuxSC8815Frequency : 300;

    logInfoP("Bus: target %u mV, limit %u mA, %u kHz, reset %u s", _bus.targetVoltage_mV, _bus.currentLimit_mA,
             _bus.frequencyKHz, _resetTimeSeconds);
    logInfoP("Aux: %s, target %u mV, limit %u mA, %u kHz, reset %u s", _aux.enabled ? "enabled" : "disabled",
             _aux.targetVoltage_mV, _aux.currentLimit_mA, _aux.frequencyKHz, _resetTimeSeconds);

    // ETS 通信对象绑定表：每侧各填一份
    buildEtsMap(_bus);
    buildEtsMap(_aux);
}

void BusPowerSupplyModule::updateUnit(BpsUnit &u, uint32_t now)
{
    // ---- 未使能（辅助电源可由 ETS 关闭）----
    if (!u.enabled)
    {
        if (u.outputOn)
            powerDownUnit(u);

        u.faultCode = 0;
        u.criticalFault = false;
        u.overCurrent = false;
        u.vbusShort = false;
        u.overTemperature = false;
        u.statusOk = false;
        return;
    }

    // ---- 外部复位按键（上拉输入, 低 = 按下）----
    updateButton(u, now);

    // ---- 过流锁存：ALERT 低 = INA238 已报警（同时 PSTOP 已被拉高）。
    //      但固件自己也会拉低这根线来强制 PSTOP（pstopForced），那种情况不是过流。
    //      这里每轮 loop 都读（ms 级），不必等 500ms 采样。
    if (!u.pstopForced && !openknx.gpio.digitalRead(u.alertPin))
    {
        if (!u.overCurrent)
            logWarningP("%s: over current latch triggered (ALERT low, PSTOP high)", u.label);

        u.overCurrent = true;
        u.faultCode |= BPS_FAULT_OVERCURRENT;
        u.criticalFault = true;
    }

    // ---- 周期采样（复位期间也继续，便于观察输出电压下降）----
    if (delayCheck(u.sampleTimer, BPS_SAMPLE_INTERVAL_MS))
    {
        u.sampleTimer = now;
        sampleUnit(u);

        // 软启动完成后恢复短路折返保护
        if (u.sfbPending && delayCheck(u.sfbTimer, BPS_OUTPUT_RAMP_MS))
        {
            if (u.chipOk)
            {
                u.sc->setSfbEnable(true);
                logDebugP("%s: short circuit fold back enabled", u.label);
            }
            u.sfbPending = false;
        }
    }

    // ---- 复位过程：输出通路保持断开，时间到后恢复 ----
    if (u.resetTimer > 0)
    {
        // 按键触发的复位：按住期间保持断开（相当于一直按着前面板按键）
        if (u.resetByButton && readButton(u))
        {
            u.resetTimer = now;
            return;
        }

        if (!delayCheck(u.resetTimer, u.resetDurationMS))
            return;

        u.resetTimer = 0;
        u.resetByButton = false;
        u.retryCount = 0;
        u.retryTimer = 0;

        // 恢复输出通路；若芯片状态不完整（例如之前因故障停过 OTG），则完整重启一次
        if (u.chipOk && u.sc->isOtgEnabled())
        {
            setOutputPath(u, true);
            logInfoP("%s: reset finished", u.label);
        }
        else
        {
            startUnit(u);
        }
        return;
    }

    // ---- 故障：关断并按退避时间重试 ----
    const bool fault = u.criticalFault || !u.chipOk || !u.inaOk;
    if (!fault)
    {
        u.retryCount = 0;
        u.retryTimer = 0;

        if (!u.outputOn)
            startUnit(u);

        return;
    }

    if (u.outputOn)
    {
        disableOutput(u);
        logWarningP("%s: output disabled (fault 0x%02X)", u.label, u.faultCode);
        u.retryTimer = now;
        return;
    }

    if (u.retryTimer == 0)
    {
        u.retryTimer = now;
        return;
    }

    const uint8_t index = (u.retryCount < cRetryDelayCount) ? u.retryCount : cRetryDelayCount - 1;
    if (delayCheck(u.retryTimer, cRetryDelays[index]))
    {
        u.retryTimer = now;
        if (u.retryCount < 255)
            u.retryCount++;

        if (startUnit(u))
            logInfoP("%s: retry successful", u.label);
    }
}

/**
 * @brief 用软件算出 VBATS 分压比，据此给出 VBAT 过压点（两组分压各自独立）
 *
 * ADIN 与 VBATS 是两套互不相干的电阻网络，各自算自己的比值：
 *   ADIN  比值 = BPS_INPUT_VOLTAGE_DIVIDER（由 BPS_ADIN_FB_* 算）-> 输入电压上报
 *   VBATS 比值 = 输入电压(ADIN) / VBATS 引脚电压（芯片 VBAT ADC）-> VBAT 过压点
 * 读不到时退回标称值 BPS_VBAT_DIVIDER（由 BPS_VBAT_FB_* 算），并与实测交叉校验。
 *
 * VBAT_SEL = External 时 VBATS 引脚电压与片内基准比较，且该保护在充/放电模式
 * 都生效，所以：   过压点(输入侧) = BPS_VBAT_REF_MV x VBATS 分压比
 *
 * ADC 要跑起来才能读，所以本函数放在采样路径里，结果存进 u.vbatOvp_mV。
 */
void BusPowerSupplyModule::checkVbatOverVoltage(BpsUnit &u)
{
    // 输入电压走 ADIN；VBATS 引脚电压走芯片自己的 VBAT ADC
    const uint32_t input_mV = (uint32_t)((float)u.sc->readAdinVoltage() * BPS_INPUT_VOLTAGE_DIVIDER);
    const uint16_t pin_mV = u.sc->readBattVoltage();

    // 软件算分压比：两个读数一比就是实际比值，比写死的常量可靠，
    // 读不到（输入太低 / 引脚没接分压）时退回标称值；
    // 标称值保留小数（如 200k/6.8k = 30.41），避免取整带来的过压点误差。
    float divider = BPS_VBAT_DIVIDER;
    if (input_mV >= 1000 && pin_mV >= 100)
    {
        const uint32_t measured = (uint32_t)((float)input_mV / (float)pin_mV + 0.5f);
        if (measured >= 2 && measured <= 200)
        {
            divider = (float)measured;

            const uint32_t diffPercent =
                (uint32_t)(fabsf(divider - BPS_VBAT_DIVIDER) * 100.0f / BPS_VBAT_DIVIDER + 0.5f);
            if (diffPercent > BPS_VBAT_PIN_TOLERANCE_PERCENT)
                logWarningP("%s: VBATS divider measured %s vs nominal %s (BPS_VBAT_FB_%sk/%sk) differ by %u%%"
                            " - update the constants",
                            u.label, oneDecimalString(divider).c_str(), oneDecimalString(BPS_VBAT_DIVIDER).c_str(),
                            oneDecimalString((float)BPS_VBAT_FB_RUP_KOHM).c_str(),
                            oneDecimalString((float)BPS_VBAT_FB_RDOWN_KOHM).c_str(), diffPercent);
        }
    }

    u.vbatDivider = (uint16_t)(divider + 0.5f);
    u.vbatOvp_mV = (uint32_t)((float)BPS_VBAT_REF_MV * divider);
    u.vbatOvpChecked = true;

    // 当前余量：引脚离片内基准还有多少（引脚电压 = 输入电压 / 分压比）
    const uint32_t pinNow_mV = (input_mV > 0 && divider > 0.0f) ? (uint32_t)((float)input_mV / divider) : 0;
    const uint32_t marginPercent =
        (pinNow_mV == 0 || pinNow_mV >= BPS_VBAT_REF_MV) ? 0 : (100 - pinNow_mV * 100 / BPS_VBAT_REF_MV);

    logInfoP("%s: VBATS divider x%s (nominal x%s = %sk/%sk), input %u mV -> VBATS %u mV of %u mV (%u%% left)"
             " => VBAT over voltage at %u mV",
             u.label, oneDecimalString(divider).c_str(), oneDecimalString(BPS_VBAT_DIVIDER).c_str(),
             oneDecimalString((float)BPS_VBAT_FB_RUP_KOHM).c_str(),
             oneDecimalString((float)BPS_VBAT_FB_RDOWN_KOHM).c_str(), input_mV, pinNow_mV, BPS_VBAT_REF_MV, marginPercent,
             u.vbatOvp_mV);

    // 过压点必须高于最高输入电压，否则输入一升高 IC 就停振、OTG 无输出
    if (u.vbatOvp_mV <= BPS_INPUT_VOLTAGE_MAX_MV)
    {
        const uint32_t rupNeeded_kOhm =
            (uint32_t)(((float)BPS_INPUT_VOLTAGE_MAX_MV / (float)BPS_VBAT_REF_MV - 1.0f) *
                       (float)BPS_VBAT_FB_RDOWN_KOHM) +
            1;
        logErrorP("%s: VBAT over voltage at %u mV is NOT above max input %u mV - input above ~%u mV kills the output!"
                  " Need Rup >= %u kOhm with Rdown %s kOhm (or lower BPS_INPUT_VOLTAGE_MAX_MV)",
                  u.label, u.vbatOvp_mV, BPS_INPUT_VOLTAGE_MAX_MV, u.vbatOvp_mV, rupNeeded_kOhm,
                  oneDecimalString((float)BPS_VBAT_FB_RDOWN_KOHM).c_str());
    }
}

void BusPowerSupplyModule::sampleUnit(BpsUnit &u)
{
    // ---- 通信检查 ----
    u.chipOk = i2cPing(*u.wire, u.chipAddress);
    u.inaOk = u.ina->isConnected();
    u.tempOk = i2cPing(*u.wire, u.tempAddress);

    // ---- 输出侧：INA238 ----
    if (u.inaOk)
    {
        u.outputVoltage_mV = u.ina->getBusVoltage() * 1000.0f;
        u.outputCurrent_mA = u.ina->getCurrent() * 1000.0f;

        u.outputLoadPercent = (u.currentLimit_mA > 0) ? (u.outputCurrent_mA * 100.0f / (float)u.currentLimit_mA) : 0.0f;
        if (u.outputLoadPercent > 100.0f) u.outputLoadPercent = 100.0f;
        if (u.outputLoadPercent < 0.0f) u.outputLoadPercent = 0.0f;
    }

    // ---- 温度：TMP102，掉线时退回 INA238 内部温度 ----
    if (u.tempOk)
        u.temperatureC = u.tmp->readTempC();
    else if (u.inaOk)
        u.temperatureC = u.ina->getTemperature();

    // ---- 输入侧与芯片状态：SC8815 ----
    if (u.chipOk)
    {
        InterruptStatus status = {};
        u.sc->readInterruptStatus(status);
        u.vbusShort = status.VBUS_SHORT;
        u.overTemperature = status.OTP;

        u.inputVoltage_mV = (float)u.sc->readAdinVoltage() * BPS_INPUT_VOLTAGE_DIVIDER;
        u.inputCurrent_mA = (float)u.sc->readBattCurrent();
    }
    else
    {
        u.vbusShort = false;
        u.overTemperature = false;
        u.inputVoltage_mV = 0.0f;
        u.inputCurrent_mA = 0.0f;
    }

    // ---- 过流锁存：读 INA238 报警标志（与 ALERT 引脚同一个片内锁存标志），
    //      再用 ALERT 引脚电平核对一次（引脚低且不是固件自己拉低的则 PSTOP 已高）----
    bool overCurrent = false;
    if (u.inaOk)
    {
        const uint16_t alert = u.ina->getDiagnoseAlert();
        overCurrent = (alert & ((1u << INA238_DIAG_SHUNT_OVER_LIMIT) | (1u << INA238_DIAG_POWER_OVER_LIMIT))) != 0;
    }
    if (!u.pstopForced && !openknx.gpio.digitalRead(u.alertPin))
        overCurrent = true;
    u.overCurrent = overCurrent;

    // ---- VBATS 分压比反推（只算一次）----
    if (u.chipOk && !u.vbatOvpChecked)
        checkVbatOverVoltage(u);

    // ---- 故障代码 ----
    uint8_t fault = 0;
    if (!u.chipOk) fault |= BPS_FAULT_SC8815_NO_ANSWER;
    if (!u.inaOk) fault |= BPS_FAULT_INA_NO_ANSWER;
    if (!u.tempOk) fault |= BPS_FAULT_TEMP_NO_ANSWER;
    if (u.vbusShort) fault |= BPS_FAULT_VBUS_SHORT;
    if (u.overTemperature) fault |= BPS_FAULT_OTP;
    if (u.overCurrent) fault |= BPS_FAULT_OVERCURRENT;
    if (u.inputVoltage_mV > 0.0f && u.inputVoltage_mV < BPS_INPUT_VOLTAGE_MIN_MV) fault |= BPS_FAULT_INPUT_LOW;
    if (u.inputVoltage_mV > BPS_INPUT_VOLTAGE_MAX_MV) fault |= BPS_FAULT_INPUT_HIGH;

    u.faultCode = fault;
    u.criticalFault = (fault & (BPS_FAULT_OVERCURRENT | BPS_FAULT_VBUS_SHORT | BPS_FAULT_OTP)) != 0;

    // ---- 电源状态（1 = 正常）----
    u.statusOk = u.outputOn && u.chipOk && !u.criticalFault &&
                 (u.outputVoltage_mV >= (float)u.targetVoltage_mV * BPS_OUTPUT_OK_PERCENT / 100.0f);
}

/* ===========================================================================
 *  ETS 通信对象
 * ======================================================================== */
/**
 * @brief 填某一侧的“通信对象 <-> ETS 参数”绑定表
 *
 * 每侧都有自己的表（u.ets），启动时各调一次：
 *     buildEtsMap(_bus);   // 走 else 分支 -> 总线侧 KO/参数
 *     buildEtsMap(_aux);   // 走 if 分支   -> 辅助侧 KO/参数
 * 所以 if/else 不是“二选一只发一侧”，而是“这次填哪张表”。
 * 两个分支只是把配对填进表里（Param* 宏不可索引，配对必须逐项写出），
 * 发送逻辑 sendUnitValues() 只有一条路径，两侧不会各维护一份代码。
 */
void BusPowerSupplyModule::buildEtsMap(BpsUnit &u)
{
    BpsEtsMap &map = u.ets;

    if (u.isAux)
    {
        map.status = {BPS_KoPowerSupply2Status, DPT_Switch, ParamBPS_PowerSupply2ChangeSend, 0, 0,
                      ParamBPS_PowerSupply2SendCyclicTimeMS};
        map.voltage = {BPS_KoAuxVoltage, DPT_Value_Electric_Potential, ParamBPS_AuxVoltageChangeSend,
                       ParamBPS_AuxVoltageSendMinChangePercent, ParamBPS_AuxVoltageSendMinChangeAbsolute,
                       ParamBPS_AuxVoltageSendCyclicTimeMS};
        map.current = {BPS_KoAuxCurrent, DPT_Value_Electric_Current, ParamBPS_AuxCurrentChangeSend,
                       ParamBPS_AuxCurrentSendMinChangePercent, ParamBPS_AuxCurrentSendMinChangeAbsolute,
                       ParamBPS_AuxCurrentSendCyclicTimeMS};
        map.load = {BPS_KoAuxLoad, DPT_Scaling, ParamBPS_AuxLoadChangeSend, ParamBPS_AuxLoadSendMinChangePercent,
                    ParamBPS_AuxLoadSendMinChangeAbsolute, ParamBPS_AuxLoadSendCyclicTimeMS};
        map.temperature = {BPS_KoTemperature2, DPT_Value_Temp, ParamBPS_AuxTemperatureChangeSend,
                           ParamBPS_AuxTemperatureSendMinChangePercent, ParamBPS_AuxTemperatureSendMinChangeAbsolute,
                           ParamBPS_AuxTemperatureSendCyclicTimeMS};
        map.inputVoltage = {BPS_KoAuxInputVoltage, DPT_Value_Electric_Potential, ParamBPS_AuxInputVoltageChangeSend,
                            ParamBPS_AuxInputVoltageSendMinChangePercent, ParamBPS_AuxInputVoltageSendMinChangeAbsolute,
                            ParamBPS_AuxInputVoltageSendCyclicTimeMS};
        map.inputCurrent = {BPS_KoAuxInputCurrent, DPT_Value_Electric_Current, ParamBPS_AuxInputCurrentChangeSend,
                            ParamBPS_AuxInputCurrentSendMinChangePercent, ParamBPS_AuxInputCurrentSendMinChangeAbsolute,
                            ParamBPS_AuxInputCurrentSendCyclicTimeMS};
        // 诊断量没有 ETS 参数：值变化就发送
        map.chipStatus = {BPS_KoAuxSC8815Status, DPT_Switch, true, 0, 0, 0};
        map.errorCode = {BPS_KoAuxSC8815ErrorCode, DPT_Value_1_Ucount, true, 0, 0, 0};
    }
    else
    {
        map.status = {BPS_KoPowerSupply1Status, DPT_Switch, ParamBPS_PowerSupply1ChangeSend, 0, 0,
                      ParamBPS_PowerSupply1SendCyclicTimeMS};
        map.voltage = {BPS_KoBusVoltage, DPT_Value_Electric_Potential, ParamBPS_BusVoltageChangeSend,
                       ParamBPS_BusVoltageSendMinChangePercent, ParamBPS_BusVoltageSendMinChangeAbsolute,
                       ParamBPS_BusVoltageSendCyclicTimeMS};
        map.current = {BPS_KoBusCurrent, DPT_Value_Electric_Current, ParamBPS_BusCurrentChangeSend,
                       ParamBPS_BusCurrentSendMinChangePercent, ParamBPS_BusCurrentSendMinChangeAbsolute,
                       ParamBPS_BusCurrentSendCyclicTimeMS};
        map.load = {BPS_KoBusLoad, DPT_Scaling, ParamBPS_BusLoadChangeSend, ParamBPS_BusLoadSendMinChangePercent,
                    ParamBPS_BusLoadSendMinChangeAbsolute, ParamBPS_BusLoadSendCyclicTimeMS};
        map.temperature = {BPS_KoTemperature, DPT_Value_Temp, ParamBPS_TemperatureChangeSend,
                           ParamBPS_TemperatureSendMinChangePercent, ParamBPS_TemperatureSendMinChangeAbsolute,
                           ParamBPS_TemperatureSendCyclicTimeMS};
        map.inputVoltage = {BPS_KoInputVoltage, DPT_Value_Electric_Potential, ParamBPS_InputVoltageChangeSend,
                            ParamBPS_InputVoltageSendMinChangePercent, ParamBPS_InputVoltageSendMinChangeAbsolute,
                            ParamBPS_InputVoltageSendCyclicTimeMS};
        map.inputCurrent = {BPS_KoInputCurrent, DPT_Value_Electric_Current, ParamBPS_InputCurrentChangeSend,
                            ParamBPS_InputCurrentSendMinChangePercent, ParamBPS_InputCurrentSendMinChangeAbsolute,
                            ParamBPS_InputCurrentSendCyclicTimeMS};
        map.chipStatus = {BPS_KoSC8815Status, DPT_Switch, true, 0, 0, 0};
        map.errorCode = {BPS_KoSC8815ErrorCode, DPT_Value_1_Ucount, true, 0, 0, 0};
    }
}

void BusPowerSupplyModule::sendValue(const BpsEtsQuantity &quantity, BpsSendState &state, float value)
{
    if (!quantity.send || quantity.ko == 0)
        return;

    GroupObject &ko = knx.getGroupObject(quantity.ko);
    const Dpt &dpt = quantity.dpt;

    const float difference = fabsf(value - state.last);

    if (difference > 0.0f)
    {
        const bool relativeOk = (state.last == 0.0f) ||
                                (difference >= fabsf(state.last) * (float)quantity.minPercent / 100.0f);
        const bool absoluteOk = (quantity.minAbsolute <= 0) || (difference >= (float)quantity.minAbsolute);

        if (relativeOk && absoluteOk)
        {
            ko.value(value, dpt);
            state.last = value;
        }
        else
        {
            // 变化太小：只更新内部值，不发送
            ko.valueNoSend(value, dpt);
        }
    }

    if (quantity.cyclicMS > 0 && delayCheckMillis(state.timer, quantity.cyclicMS))
    {
        ko.value(value, dpt);
        state.last = value;
        state.timer = delayTimerInit();
    }
}

void BusPowerSupplyModule::sendUnitValues(BpsUnit &u)
{
    const float chipStatus = (u.chipOk && !u.vbusShort && !u.overTemperature) ? 1.0f : 0.0f;
    const BpsEtsMap &map = u.ets;

    sendValue(map.status, u.sendStatus, u.statusOk ? 1.0f : 0.0f);
    sendValue(map.voltage, u.sendVoltage, u.outputVoltage_mV);
    sendValue(map.current, u.sendCurrent, u.outputCurrent_mA);
    sendValue(map.load, u.sendLoad, u.outputLoadPercent);
    sendValue(map.temperature, u.sendTemperature, u.temperatureC);
    sendValue(map.inputVoltage, u.sendInputVoltage, u.inputVoltage_mV);
    sendValue(map.inputCurrent, u.sendInputCurrent, u.inputCurrent_mA);
    sendValue(map.chipStatus, u.sendChipStatus, chipStatus);
    sendValue(map.errorCode, u.sendFaultCode, (float)u.faultCode);
}

/* ===========================================================================
 *  输出/控制台
 * ======================================================================== */

void BusPowerSupplyModule::logUnit(BpsUnit &u)
{
    logInfoP("%s: %s | out %u mV / %u mA / %u %% | in %u mV / %u mA | %s C | fault 0x%02X", u.label,
             u.outputOn ? "on " : "off", (uint16_t)(u.outputVoltage_mV + 0.5f), (uint16_t)(u.outputCurrent_mA + 0.5f),
             (uint8_t)(u.outputLoadPercent + 0.5f), (uint16_t)(u.inputVoltage_mV + 0.5f),
             (uint16_t)(u.inputCurrent_mA + 0.5f), oneDecimalString(u.temperatureC).c_str(), u.faultCode);
}

void BusPowerSupplyModule::printUnit(BpsUnit &u, bool diagnoseKo)
{
    const char *state = u.outputOn ? "on" : "off";

    if (diagnoseKo)
    {
        openknx.console.writeDiagenoseKo(
            "%s: %s, out %u mV / %u mA / %u %%, in %u mV / %u mA, %s C, fault 0x%02X", u.label, state,
            (uint16_t)(u.outputVoltage_mV + 0.5f), (uint16_t)(u.outputCurrent_mA + 0.5f),
            (uint8_t)(u.outputLoadPercent + 0.5f), (uint16_t)(u.inputVoltage_mV + 0.5f),
            (uint16_t)(u.inputCurrent_mA + 0.5f), oneDecimalString(u.temperatureC).c_str(), u.faultCode);
        return;
    }

    logInfoP("%s: output %s (target %u mV, limit %u mA, %u kHz, reset %u s)", u.label, state, u.targetVoltage_mV,
             u.currentLimit_mA, u.frequencyKHz, _resetTimeSeconds);
    logInfoP("%s: out %u mV, %u mA, %u %% load", u.label, (uint16_t)(u.outputVoltage_mV + 0.5f),
             (uint16_t)(u.outputCurrent_mA + 0.5f), (uint8_t)(u.outputLoadPercent + 0.5f));
    logInfoP("%s: in  %u mV, %u mA, %s C", u.label, (uint16_t)(u.inputVoltage_mV + 0.5f),
             (uint16_t)(u.inputCurrent_mA + 0.5f), oneDecimalString(u.temperatureC).c_str());
    logInfoP("%s: SC8815 %s, INA238 %s, sensor %s, fault 0x%02X, retries %u, button %s, PSTOP %s", u.label,
             u.chipOk ? "ok" : "missing", u.inaOk ? "ok" : "missing", u.tempOk ? "ok" : "missing", u.faultCode,
             u.retryCount, u.buttonPressed ? "pressed" : "released", u.pstopForced ? "forced high" : "released");

    // VBAT 过压点（由 VBATS 分压比决定）与当前输入的余量：
    // 引脚一旦碰到 BPS_VBAT_REF_MV, IC 就停振、输出没了, 所以余量就是安全裕度。
    if (u.vbatOvp_mV > 0)
    {
        const uint32_t input_mV = (uint32_t)u.inputVoltage_mV;
        const uint32_t margin_mV = (u.vbatOvp_mV > input_mV) ? (u.vbatOvp_mV - input_mV) : 0;
        logInfoP("%s: VBAT over voltage at %u mV (VBATS divider x%u), input %u mV -> margin %u mV", u.label,
                 u.vbatOvp_mV, u.vbatDivider, input_mV, margin_mV);
    }
}

uint32_t BusPowerSupplyModule::resetDurationMS(const BpsUnit &u) const
{
    // share.xml 只有一个“复位时间”参数, 总线与辅助共用
    (void)u;
    return (uint32_t)_resetTimeSeconds * 1000;
}

void BusPowerSupplyModule::showInformations()
{
    printUnit(_bus, false);
    printUnit(_aux, false);
}

void BusPowerSupplyModule::showHelp()
{
    openknx.console.printHelpLine("bps", "显示总线电源模块的帮助");
    openknx.console.printHelpLine("bps info", "显示两路输出的测量值与状态");
    openknx.console.printHelpLine("bps bus on|off", "打开/关闭总线电源输出");
    openknx.console.printHelpLine("bps aux on|off", "打开/关闭辅助电源输出");
    openknx.console.printHelpLine("bps bus|aux reset", "复位该路输出（时长取 ETS“复位时间”；前面板按键需按住 3 s 以上）");
    openknx.console.printHelpLine("bps bus|aux clear", "清除过流锁存并重新启动该路输出");
    openknx.console.printHelpLine("bps bus|aux pstop on|off", "硬件强制停/释放 PSTOP（直接拉低/释放 ALERT 引脚）");
    openknx.console.printHelpLine("bps bus|aux info", "显示该路输出的测量值与状态");
    openknx.console.printHelpLine("bps test", "周期打印测量值（测试模式开关）");
}

bool BusPowerSupplyModule::processCommand(const std::string cmd, bool diagnoseKo)
{
    if (cmd.compare(0, 3, "bps") != 0)
        return false;

    if (cmd.length() == 3)
    {
        showHelp();
        return true;
    }

    const std::string arguments = cmd.substr(4);
    if (arguments == "info" || arguments == "status")
    {
        printUnit(_bus, diagnoseKo);
        printUnit(_aux, diagnoseKo);
        return true;
    }

    if (arguments == "test")
    {
        _testMode = !_testMode;
        logInfoP("Test mode %s", _testMode ? "on" : "off");
        return true;
    }

    BpsUnit *unit = nullptr;
    std::string action;
    if (arguments.rfind("bus", 0) == 0)
    {
        unit = &_bus;
        action = arguments.length() > 3 ? arguments.substr(4) : "";
    }
    else if (arguments.rfind("aux", 0) == 0)
    {
        unit = &_aux;
        action = arguments.length() > 3 ? arguments.substr(4) : "";
    }

    if (unit == nullptr)
    {
        logInfoP("bps: bad arguments");
        if (diagnoseKo)
            openknx.console.writeDiagenoseKo("bps: bad arguments");
        showHelp();
        return true;
    }

    if (action == "on")
    {
        unit->enabled = true;
        startUnit(*unit);
    }
    else if (action == "off")
    {
        unit->enabled = false;
        powerDownUnit(*unit);
        logInfoP("%s: output disabled by console", unit->label);
    }
    else if (action == "reset")
    {
        // 复位时长来自 ETS 参数 BPS_ResetTime（share.xml 只有这一个，总线/辅助共用）
        startReset(*unit, resetDurationMS(*unit));
    }
    else if (action == "clear")
    {
        clearLatch(*unit);
        startUnit(*unit);
        logInfoP("%s: over current latch cleared", unit->label);
    }
    else if (action == "pstop on")
    {
        // 主动硬停：拉低 ALERT -> 触发器把 PSTOP 拉高（与过流同一条硬件通路）
        setPstop(*unit, true);
        setOutputPath(*unit, false);
        logInfoP("%s: PSTOP forced high (ALERT held low)", unit->label);
    }
    else if (action == "pstop off")
    {
        // 释放 + 清锁存（只释放不脉冲 CLR 的话, 触发器仍然是锁存的, PSTOP 不会回低）
        clearLatch(*unit);
        logInfoP("%s: PSTOP released (latch cleared)", unit->label);
    }
    else if (action == "info" || action == "status")
    {
        printUnit(*unit, diagnoseKo);
    }
    else
    {
        logInfoP("bps: bad arguments");
        if (diagnoseKo)
            openknx.console.writeDiagenoseKo("bps: bad arguments");
        showHelp();
    }

    return true;
}
