#pragma once

#include "OpenKNX.h"
#include "hardware.h"
#include "knxprod.h"

#include <Wire.h>
#include <INA238.h>
#include <SparkFunTMP102.h>
#include <sc8815.h>

/* ============================================================================
 *  ETS 模块 "SC8815 电源" (BusPowerSupply)
 * ----------------------------------------------------------------------------
 *  硬件 (BOARD_SC8815_SMARTHOUSE_BPS_REG4)：两块 SC8815 各自独立，各有自己的
 *  I2C 总线与测量/保护器件：
 *
 *    总线侧 : SC8815 + INA238 + TMP102  ->  OPENKNX_GPIO_WIRE  (SDA4 / SCL5)
 *    辅助侧 : SC8815 + INA238 + TMP102  ->  OPENKNX_GPIO_WIRE1 (SDA14 / SCL15)
 *
 *  每块 SC8815 工作在放电(OTG)模式：VBAT（输入侧电源轨）-> VBUS（输出侧）
 *
 *    输出电压 / 输出电流 / 负载 : 输出侧 INA238（分流 = OPENKNX_BPS_INA_*_SHUNT）
 *    输入电压                   : SC8815 ADIN 引脚（板上 200k/10k 分压 + 电压跟随器）
 *    输入电流                   : SC8815 内部 ADC（IBAT，即 VBAT 侧）
 *    温度                       : TMP102（SC8815 无温度回读寄存器；
 *                                 若 TMP102 掉线则退回 INA238 内部温度）
 *
 *  过流保护回路（每侧独立。INA238 的 ALERT 是一根线三个作用）：
 *
 *    INA238 ALERT (开漏, 低有效, 片内锁存)
 *      |-> OPENKNX_BPS_INA_*_ALERT_PIN  MCU GPIO：**既读又写**（仿开漏）
 *      |     固件读低 = INA238 已报警（硬件已把 PSTOP 拉高、输出已断）
 *      |     固件拉低 = 主动把 PSTOP 拉高（功率级硬停，见 setPstop()）
 *      |     固件释放 = 交回上拉，节点电平由 INA238 决定
 *      |     注意: 只能“拉低 / 释放”，不能推挽驱高（INA238 是开漏输出）
 *      \-> SN74LVC1G74 的 PRE (CLK = D = GND)
 *              SN74LVC1G74 的 Q --> SC8815 的 PSTOP
 *        => 输出过流 -> ALERT 拉低 -> 硬件立刻把 PSTOP 拉高, 功率级停止并保持(锁存)
 *           固件拉低 ALERT 也能硬停；两个来源的锁存都只能靠 clearLatch() 解除。
 *           手册要求标 [PSTOP] 的寄存器（VBAT_SET / RATIO / CTRL*）必须在
 *           PSTOP 为高时写入，所以 startUnit() 的顺序是:
 *               setPstop(true) -> configureChip() -> clearLatch() -> 开 PGATE
 *
 *    SN74LVC1G74 的 CLR          --> OPENKNX_BPS_SC8815_*_CLR_PIN (软件清除锁存)
 *    OPENKNX_BPS_SC8815_*_CE_PIN --> SC8815 芯片使能 (低有效)
 *    OPENKNX_BPS_SC8815_*_RESET_PIN --> 外部复位按键输入 (上拉, 低 = 按下)
 *        必须按住 BPS_BUTTON_HOLD_MS (3 s) 以上才触发该路“复位”, 短按忽略;
 *        触发后断开输出, 松开起计 ETS“复位时间”(总线/辅助共用同一个参数)后恢复;
 *        一直按住则一直保持断开
 *    输出通断由 SC8815 的 PGATE 引脚控制 (驱动外部 PMOS 栅极):
 *        setPgateEnable(true) = 接通输出, setPgateEnable(false) = 断开输出
 *    OPENKNX_BPS_SC8815_*_INT_PIN   --> SC8815 中断输出 (开漏, 上拉, 目前未使用;
 *        故障均通过 I2C 读 SC8815 中断状态寄存器获取, 见 sampleUnit())
 *
 *  参数与通信对象定义见 SC8815PowerSupplyModule.share.xml，宏名见 knxprod.h。
 * ========================================================================== */

// ---------------------------------------------------------------------------
//  运行参数
// ---------------------------------------------------------------------------
#define BPS_SAMPLE_INTERVAL_MS 500       // 采样周期
#define BPS_CHIP_STARTUP_MS 10           // CE 使能后等待芯片内部 LDO 稳定
#define BPS_OUTPUT_RAMP_MS 100           // 释放输出后等待软启动, 再恢复短路折返保护
#define BPS_LATCH_CLEAR_US 5000          // 锁存器 CLR 脉冲宽度
#define BPS_BUTTON_DEBOUNCE_MS 30        // 外部复位按键去抖时间
#define BPS_BUTTON_HOLD_MS 3000          // 外部复位按键长按触发时间（不足 3 s 的短按不动输出）

// ---------------------------------------------------------------------------
//  电气配置（必须与实际硬件一致）
// ---------------------------------------------------------------------------
#define BPS_BUS_TARGET_VOLTAGE_MV 29000  // KNX 总线额定输出电压（无 ETS 参数, 固定值）

// ---- 输入电压检测（VBAT 输入 18..32 V）--------------------------------------
// 板上 VBAT 输入经 200k / 10k 分压（+ 电压跟随器）接 SC8815 的 ADIN 引脚，
// ADIN 量程 0..2048mV / 分辨率 2mV，因此：
//     输入电压 = ADIN 原始读数 x BPS_INPUT_VOLTAGE_DIVIDER(21)   <- 总线/辅助输入
//     电压 KO 上报的就是这个值（见 sampleUnit()）
// 可测上限 = 2048mV x 21 = 43 V，覆盖 32 V 有余。
// 注意: ADIN 与 VBATS 是**两套互相独立的分压网络**, 各自的电阻在下面分别填。
#define BPS_ADIN_FB_RUP_KOHM 200.0   // ADIN 分压上电阻
#define BPS_ADIN_FB_RDOWN_KOHM 10.0  // ADIN 分压下电阻
#define BPS_INPUT_VOLTAGE_DIVIDER (1.0f + (float)BPS_ADIN_FB_RUP_KOHM / (float)BPS_ADIN_FB_RDOWN_KOHM) // = 21

// 输入欠压/过压门限取额定范围(18..32 V)端点：
//   低于 18 V -> BPS_FAULT_INPUT_LOW ，高于 32 V -> BPS_FAULT_INPUT_HIGH
#define BPS_INPUT_VOLTAGE_MIN_MV 18000   // 输入欠压门限（VBAT 最低 18 V）
#define BPS_INPUT_VOLTAGE_MAX_MV 32000   // 输入过压门限（VBAT 最高 32 V）
#define BPS_OUTPUT_OK_PERCENT 90         // 输出电压达到目标值的该百分比即视为正常

#define BPS_INA_MAX_CURRENT_A 5.0f       // INA238 分流标定量程（需满足 maxA x shunt <= 163.84mV）
#define BPS_OVERCURRENT_ALERT_PERCENT 150 // INA238 过流报警门限 = 设定限流 x 1.5
#define BPS_INPUT_CURRENT_LIMIT_MAX_MA 3000 // 输入侧(IBAT)限流上限（SC8815 内部 ADC 可到 12A，此处按板子取上限）

// SC8815 采样电阻（VBUS 侧 RS1 / VBAT 侧 RS2）
#define BPS_SHUNT_MOHM 10

// VBAT(VBATS) 侧：本设计 VBAT 是输入侧电源轨（18..32 V）而不是电池。
// 必须使用 External，否则内部设定（最大 4S x 4.5V = 18V）的 VBAT 过压保护
// 会在输入电压超过 18 V 时立即误动作。
//
// VBATS 外部分压（注意：与下面 BPS_FB_* 的 VBUS 反馈分压、以及 ADIN 的
// 200k/10k 是三个完全不同的网络，不要混用）。
//
// 手册原文（VBAT_SET / VBAT_SEL）：
//   内部设定 (VBAT_SEL = 0): VBATS 引脚接 VBAT 端感测电池电压, 目标电压由
//       CSEL(串数) + VCELL_SET(单节电压) 决定; 例 2S + 4.3V/节 -> CSEL=01, VCELL_SET=011。
//   外部设定 (VBAT_SEL = 1): VBATS 引脚接电阻分压设定目标电压, **CSEL / VCELL_SET
//       完全不起作用**（手册原文: "VCELL_SET and CSEL bits don't work"）, 参考基准 1.2V:
//
//           VBAT = 1.2V x (1 + Rup/Rdown)   <-- 这就是 VBAT 过压点在多少伏
//
//   且 VBAT 过压保护在充电与放电(OTG)模式都生效 (手册 8.11.2) => 分压比必须按
//   最高输入电压设计，否则输入一升到门限以上 IC 就停振、OTG 无输出。
//
//   设计下限: > 32V（最高输入）  =>  (1 + Rup/Rdown) > 26.7。
//
//   已按 18..32 V 输入重算（上电阻保持 200k，只改下电阻）：
//     200k / 6.8k => x30.41 => 过压点 36.49 V（对 32 V 留 14% 余量）
//     1% 电阻最坏情况 x29.8..31.0 => 35.8..37.2 V，仍高于 32 V。
//   对照表（Rup 固定 200k，E24 取值）:
//     8.2k -> x25.4 -> 30.5 V   仍不够
//     7.5k -> x27.7 -> 33.2 V   勉强（最坏情况碰到 32 V）
//     6.8k -> x30.4 -> 36.5 V   <- 采用
//     6.2k -> x33.3 -> 39.9 V
//     5.6k -> x36.7 -> 44.1 V
//   上限另有限制: 引脚电压 = 输入/分压比 必须 <= ADIN 量程 2048mV
//     （即分压比 >= 15.6，本方案 x30.4，32 V 时引脚 1052 mV，余量充足）
//
// VBATS 分压：
//     过压点(输入侧) = BPS_VBAT_REF_MV(1.2V) x BPS_VBAT_DIVIDER
// 分压比以软件算出来的为准（见 checkVbatOverVoltage()）：
//     实测反推：VBATS 分压比 = 输入电压(ADIN) / VBATS 引脚电压(芯片 VBAT ADC)
//     读不到时退回按电阻算的标称值 BPS_VBAT_FB_*
// 两者差超过 BPS_VBAT_PIN_TOLERANCE_PERCENT 会告警（常量与板子不符）。
// 输入电压由 ADIN 读出，引脚电压 = 输入电压 / 分压比，两者一比就知道离跳闸还有多少余量。
// BPS_VBAT_REF_MV 是唯一无法测出来的量（芯片内部基准 1.2V），取自手册。
#define BPS_VBAT_SEL SCBAT_VBAT_SEL_External
#define BPS_VBAT_CSEL SCBAT_CSEL_4S    // 外部设定下不起作用（保留以随驱动一并下发）
#define BPS_VBAT_VCELL SCBAT_VCELL_4v2 // 外部设定下不起作用（保留以随驱动一并下发）
#define BPS_VBAT_REF_MV 1200           // 手册: VBATS 外部分压的参考基准 = 1.2V
#define BPS_VBAT_FB_RUP_KOHM 200.0     // VBATS 分压上电阻（独立于 BPS_ADIN_FB_*）
#define BPS_VBAT_FB_RDOWN_KOHM 6.8     // VBATS 分压下电阻（按 18..32 V 输入算, 见上）
#define BPS_VBAT_DIVIDER (1.0f + (float)BPS_VBAT_FB_RUP_KOHM / (float)BPS_VBAT_FB_RDOWN_KOHM)
#define BPS_VBAT_PIN_TOLERANCE_PERCENT 15 // 标称与实测分压比允许的偏差（超过则告警）
// VBUS 反馈：输出需要 24~30V, 内部基准(VBUSREF_I)最大只能到约 25.6V,
// 因此必须使用外部 FB 分压。Rup / Rdown 必须与板上实际分压一致!
//
// 板上 200k / 13.3k => 分压比 1 + 200/13.3 = 16.04 =>
//   最高输出 = 2048mV x 16.04 = 32.85 V（总线 29 V 与辅助 24..30 V 都有着落）
//   电压步进 = 2mV x 16.04 = 32 mV（10 bit DAC, 量化误差 ±16 mV）
//   DAC 码: 29 V -> 904 (实测 28.996 V)，24 V -> 748，30 V -> 935，上限 1024
// 注: 若以后要把辅助电压枚举值往 30 V 以上扩, 这个分压还能到 32.8 V，最多到码 1024。
// (启动日志会打印由此算出的最高输出电压与实际 DAC 值, 可据此校验)
#define BPS_VBUS_FB_MODE SCHWI_FB_External
#define BPS_FB_RUP_KOHM 200.0
#define BPS_FB_RDOWN_KOHM 13.3

// ---------------------------------------------------------------------------
//  通信对象"故障代码"的位定义（0 = 无故障）
// ---------------------------------------------------------------------------
enum BpsFault : uint8_t
{
    BPS_FAULT_SC8815_NO_ANSWER = 0x01, // SC8815 无 I2C 应答
    BPS_FAULT_INA_NO_ANSWER = 0x02,    // INA238 无 I2C 应答
    BPS_FAULT_TEMP_NO_ANSWER = 0x04,   // 温度传感器无 I2C 应答
    BPS_FAULT_VBUS_SHORT = 0x08,       // SC8815 报告输出短路
    BPS_FAULT_OTP = 0x10,              // SC8815 过温保护
    BPS_FAULT_OVERCURRENT = 0x20,      // 输出过流（硬件锁存, PSTOP 被拉高）
    BPS_FAULT_INPUT_HIGH = 0x40,       // 输入电压过高
    BPS_FAULT_INPUT_LOW = 0x80,        // 输入电压过低
};

// 某个通信对象的发送状态
struct BpsSendState
{
    float last = 0.0f; // 上次发送的值
    uint32_t timer = 0;
};

// 一个通信对象的发送配置（KO 编号 + 数据类型 + 发送条件）
struct BpsEtsQuantity
{
    uint16_t ko = 0;         // KO 编号（BPS_Ko...），0 = 未使用
    Dpt dpt;                 // 数值类型（default 构造为无效值，统一在表中指定）
    bool send = false;       // ETS：是否发送
    uint8_t minPercent = 0;  // ETS：相对最小变化量（%）
    int16_t minAbsolute = 0; // ETS：绝对最小变化量
    uint32_t cyclicMS = 0;   // ETS：周期发送时间（ms，0 = 不周期发送）
};

// 一路输出的全部 ETS 绑定（每侧一份，启动时由 buildEtsMap() 填好）
struct BpsEtsMap
{
    BpsEtsQuantity status;       // 电源状态
    BpsEtsQuantity voltage;      // 输出电压
    BpsEtsQuantity current;      // 输出电流
    BpsEtsQuantity load;         // 输出负载
    BpsEtsQuantity temperature;  // 温度
    BpsEtsQuantity inputVoltage; // 输入电压
    BpsEtsQuantity inputCurrent; // 输入电流
    BpsEtsQuantity chipStatus;   // SC8815 运行状态
    BpsEtsQuantity errorCode;    // 故障代码
};

// 一侧（总线或辅助）的全部运行状态
struct BpsUnit
{
    // ---- 硬件（构造时绑定）----
    const char *label = "";
    bool isAux = false;
    TwoWire *wire = nullptr;
    uint8_t chipAddress = SC8815_I2C_ADDR;
    uint8_t tempAddress = 0x48;
    uint8_t intPin = 255;
    uint8_t clrPin = 255;
    uint8_t cePin = 255;
    uint8_t resetPin = 255;
    uint8_t alertPin = 255;
    uint8_t inaAddress = OPENKNX_BPS_INA_BUS_POWER_SUPPLY_ADDR; // INA238 地址
    float inaShuntOhm = OPENKNX_BPS_INA_BUS_POWER_SUPPLY_SHUNT; // INA238 输出侧采样电阻（Ω）
    uint8_t clrActive = LOW;
    uint8_t ceActive = LOW;
    uint8_t resetActive = LOW;
    SC8815 *sc = nullptr;
    INA238 *ina = nullptr;
    TMP102 *tmp = nullptr;

    // ---- ETS 目标值 ----
    bool enabled = false;
    uint16_t targetVoltage_mV = 0;
    uint16_t currentLimit_mA = 0;
    uint16_t frequencyKHz = 300;

    // ---- 测量值（与通信对象同单位: mV / mA / % / °C）----
    float outputVoltage_mV = 0.0f;
    float outputCurrent_mA = 0.0f;
    float outputLoadPercent = 0.0f;
    float inputVoltage_mV = 0.0f;
    float inputCurrent_mA = 0.0f;
    float temperatureC = 0.0f;

    // ---- 状态 ----
    bool chipOk = false;
    bool inaOk = false;
    bool tempOk = false;
    bool outputOn = false;
    bool statusOk = false;
    bool overCurrent = false;
    bool vbusShort = false;
    bool overTemperature = false;
    bool criticalFault = false;
    bool sfbPending = false;
    uint8_t faultCode = 0;

    // ---- 输入侧 VBATS 分压/过压点（软件算出，见 checkVbatOverVoltage()）----
    uint32_t vbatOvp_mV = 0;      // 算出的 VBAT 过压点；0 = 还没算出来
    uint16_t vbatDivider = 0;     // 软件算出的 VBATS 分压比（实际用的那个）
    bool vbatOvpChecked = false;  // 已经算过一次（不管成功与否）

    // ---- PSTOP（由固件主动拉低 ALERT 强制拉高，见 setPstop()）----
    bool pstopForced = false; // true = 当前是固件把 ALERT 拉低（不是 INA238 报警）

    // ---- 外部复位按键 ----
    bool buttonPressed = false;       // 去抖后的按键状态
    bool buttonLast = false;          // 上一次原始电平
    bool buttonHoldTriggered = false; // 本次长按已触发过复位（松开后清零）
    bool resetByButton = false;       // 本次复位由按键触发(按住期间保持复位)

    // ---- 计时 ----
    uint32_t sampleTimer = 0;
    uint32_t retryTimer = 0;
    uint32_t resetTimer = 0;
    uint32_t resetDurationMS = 0;
    uint32_t sfbTimer = 0;
    uint32_t buttonTimer = 0;
    uint32_t buttonHoldTimer = 0;     // 长按计时起点（去抖通过那一刻）
    uint8_t retryCount = 0;

    // ---- 通信对象发送状态 ----
    BpsSendState sendStatus;
    BpsSendState sendVoltage;
    BpsSendState sendCurrent;
    BpsSendState sendLoad;
    BpsSendState sendTemperature;
    BpsSendState sendInputVoltage;
    BpsSendState sendInputCurrent;
    BpsSendState sendChipStatus;
    BpsSendState sendFaultCode;

    // ---- 本侧的通信对象绑定表（启动时由 buildEtsMap() 填好）----
    BpsEtsMap ets;
};

class BusPowerSupplyModule : public OpenKNX::Module
{
  public:
    BusPowerSupplyModule();

    const std::string name() override;
    const std::string version() override;

    void setup(bool configured) override;
    void loop(bool configured) override;
    void processInputKo(GroupObject &ko) override;

    bool processCommand(const std::string cmd, bool diagnoseKo) override;
    void showHelp() override;
    void showInformations() override;

  private:
    // 总线侧器件（Wire）
    SC8815 _scBus{OPENKNX_GPIO_WIRE, SC8815_I2C_ADDR};
    INA238 _inaBus{OPENKNX_BPS_INA_BUS_POWER_SUPPLY_ADDR, &OPENKNX_GPIO_WIRE};
    TMP102 _tmpBus;
    // 辅助侧器件（Wire1）
    SC8815 _scAux{OPENKNX_GPIO_WIRE1, SC8815_I2C_ADDR};
    INA238 _inaAux{OPENKNX_BPS_INA_AUX_POWER_SUPPLY_ADDR, &OPENKNX_GPIO_WIRE1};
    TMP102 _tmpAux;

    BpsUnit _bus;
    BpsUnit _aux;

    uint8_t _resetTimeSeconds = 10; // ETS“复位时间”，总线/辅助共用（share.xml 只有 00001）
    bool _testMode = false;
    uint32_t _testTimer = 0;

    // ---- 硬件与控制 ----
    void initPins(BpsUnit &u);
    void setOutputPath(BpsUnit &u, bool enable);
    bool readButton(BpsUnit &u);
    void updateButton(BpsUnit &u, uint32_t now);
    bool configureChip(BpsUnit &u);
    void setPstop(BpsUnit &u, bool high); // 用 ALERT 引脚把 PSTOP 拉高/释放（仿开漏）
    void clearLatch(BpsUnit &u);
    bool startUnit(BpsUnit &u);
    void disableOutput(BpsUnit &u);
    void powerDownUnit(BpsUnit &u);
    void startReset(BpsUnit &u, uint32_t durationMS, bool fromButton = false);

    // ---- 运行 ----
    void applyParameters(bool configured);
    void buildEtsMap(BpsUnit &u); // 只填传入这一侧的表：_bus 填总线, _aux 填辅助
    void updateUnit(BpsUnit &u, uint32_t now);
    void sampleUnit(BpsUnit &u);
    void checkVbatOverVoltage(BpsUnit &u); // 读 SC8815 反推 VBATS 分压比并校验过压点

    // ---- ETS ----
    void sendUnitValues(BpsUnit &u);
    void sendValue(const BpsEtsQuantity &quantity, BpsSendState &state, float value);

    // ---- 输出 ----
    void logUnit(BpsUnit &u);
    void printUnit(BpsUnit &u, bool diagnoseKo);
    uint32_t resetDurationMS(const BpsUnit &u) const;
};

extern BusPowerSupplyModule openknxBusPowerSupplyModule;
