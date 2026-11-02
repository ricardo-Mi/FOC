# V4 闭环速度 + 电流采样（快照）

- 快照时间：2026-10-26
- 对应代码：工作区 `src/`、`inc/` 中参与编译的 24 个文件（逐文件复制，SHA256 与源文件一致）
- 定位：在 V3 速度闭环的基础上，接入 DRV8323 三相低边电流采样，并把电流纳入启动流程和故障保护

这份目录只用于留档，不参与编译（`CMakeLists.txt` 的 `APP_SOURCES` 不引用它）。
其中不含 `Drivers/`、`FreeRTOS/`、`CMakeLists.txt`，不能单独构建。

## 与 V3 的主要差异

| 项目 | V3_闭环速度FOC | V4（本快照） |
|---|---|---|
| 功率级 | TIM3 三路单端 PWM：PA6/PA7/PD8，使能 PA10 | TIM1 三相互补 PWM：PE9/PE8、PE11/PE10、PA10/PE12，0.5 us 死区，使能 PE7 |
| 电流采样 | 无 | ADC1/2/3 注入组同步采样 PA4/PA5/PA3，TIM1_TRGO 触发 |
| 启动流程 | 直接进入速度闭环 | STOP→WAKE→CAL→OFST→ALGN→RUN，含两段零点校准 |
| 故障保护 | 无电流判断 | RUN/ALGN 中连续 5 次（5 ms）电流无效锁存 ERROR |
| 速度环参数 | KP 0.08、KI 0.30、无限幅斜率限制之外无额外滤波说明 | KP 0.035、KI 0.45、KD 0、Tf 30 ms，PID ramp 0 |
| 遥测 | USART1 调试（已停用） | USART3 PD8，FireWater 8 通道，100 Hz |
| 按键 | Key_device 多键扫描（5 个键） | PC0 单键，10 ms 边沿检测 |
| 其它外设 | LED、队列/信号量/软件定时器 | 已移除，只保留 FOC 所需外设 |
| 源码注释 | GBK 中文 | ASCII 英文（避免 GBK 在 GCC/编辑器里的乱码） |

## 文件清单

| 文件 | 作用 |
|---|---|
| `main.c` | 系统初始化顺序、任务创建、FreeRTOS 钩子 |
| `FOC.c` / `FOC.h` | 状态机、速度环、反 Park/Clarke、TIM1 PWM、电角度对齐、电流故障锁存 |
| `ADC.c` / `ADC.h` | 三相同步电流采样、DRV8323 CAL 零点校准、PWM 开启后二次零点、共模扣除、采样有效性 |
| `PID.c` / `PID.h` | Tustin 积分 + 输出/积分限幅 + 可选输出斜率 |
| `LowPassFilter.c` / `LowPassFilter.h` | 基于 `micros()` 的一阶低通（dt 自适应，长间隔自动重启） |
| `AS5600.c` / `AS5600.h` | PB13/PB12 软件 I2C 读 12 位角度并差分测速 |
| `Key.c` / `Key.h` | PC0 启停按键 |
| `Board.c` / `Board.h` | PA2 DRV8323 故障输入 |
| `OLED.c` / `OLED.h` / `OLED_Font.h` | 三相电流、状态、故障位显示 |
| `VOFA.c` / `VOFA.h` | FireWater 帧格式化 + 100 Hz 任务 |
| `VOFA_UART.c` / `VOFA_UART.h` | USART3 PD8 发送 |
| `Delay.c` / `Delay.h` | DWT 微秒计时/延时 |

## 引脚与外设

| 功能 | 引脚 | 说明 |
|---|---|---|
| A 相 PWM | PE9 / PE8 | TIM1_CH1 / CH1N |
| B 相 PWM | PE11 / PE10 | TIM1_CH2 / CH2N |
| C 相 PWM | PA10 / PE12 | TIM1_CH3 / CH3N |
| PWM 频率 | — | 168 MHz / 22 / 256 ≈ 29.8 kHz，死区 84 tick = 0.5 us |
| DRV8323 使能 | PE7 | 高有效 |
| DRV8323 CAL | PE5 | 高电平约 200 us 触发内部失调校准 |
| A/B/C 相电流 | PA4 / PA5 / PA3 | ADC1_IN4 / ADC2_IN5 / ADC3_IN3，注入组同步 |
| 编码器 | PB13 SCL / PB12 SDA | AS5600，7 位地址 0x36，方向 -1 |
| 启停按键 | PC0 | 上拉输入，低有效（10 ms 扫描） |
| 故障输入 | PA2 | 上拉输入，读回值上报 OLED |
| OLED | PF9 / PF10 | 软件 I2C |
| 遥测 | PD8 | USART3 TX，115200 8N1，FireWater |

## 关键参数（与代码一致）

速度环（`src/FOC.c`）：

```text
FOC_SUPPLY_VOLTAGE   = 12.6 V      -> Uq 限幅 ±6.3 V
FOC_POLE_PAIRS       = 7
FOC_DEFAULT_TARGET   = 15.0 rad/s  (机械角速度)
FOC_VEL_KP           = 0.035
FOC_VEL_KI           = 0.45
FOC_VEL_KD           = 0.0
FOC_VEL_LPF_TF       = 0.030 s     (30 ms)
PID_Init(... ramp = 0.0, limit = Vbus/2)
```

对齐与保护：

```text
FOC_ALIGN_VOLTAGE    = 3.0 V
FOC_ALIGN_TIME_MS    = 3000 ms
FOC_ALIGN_ANGLE      = 1.5 * pi
FOC_CURRENT_FAULT_COUNT = 5        (5 ms 连续无效才锁存 ERROR)
```

电流换算（`src/ADC.c`）：

```text
I = (offset - raw) * ADC_PHASE_SCALE
ADC_PHASE_SCALE = 3.3 / (4095 * 0.004 * 5.0) ≈ 0.0403 A/count
样本时间 84 cycles，ADCCLK = 84 MHz / 4 = 21 MHz
OFFSET_SAMPLES = 1024（两次：PWM 关闭时 + PWM 开启后）
共模估计：common = (Ia+Ib+Ic)/3，低通 Tf = 20 ms，从三相中扣除
```

## 启动/运行时序

```text
STOP ──按 PC0──► WAKE ─300ms─► 使能 DRV8323
                    └─+5ms─► CAL：PE5 拉高 200us，等 5 ms，采 1024 点求零点
                              └─就绪─► 中性占空比 + 使能 MOE ─► OFST
                                        OFST：等 10 ms，带 PWM 再采 1024 点零点
                                          ├─已对齐过─► RUN
                                          └─未对齐──► ALGN：3 s 拉到 1.5π，记录零点电角度 ─► RUN
RUN：Uq = PID(target - measured_velocity)，angle_el = 7*angle_mech - zero
ERROR：按一次 PC0 清故障，再按一次重新启动
```

- 每次 `WAKE`/`OFST` 等待超时 1 s，超时进 ERROR。
- `FOC_MotorDisable()` 会关 MOE、拉低 PE7、复位 PID/LPF、把电流模块置为无效态。

## 任务与周期

| 任务 | 栈（word） | 优先级 | 周期 |
|---|---|---|---|
| `vFOCSpeedTask` | 384 | 3 | 1 ms（1 kHz） |
| `vKeyTask` | 128 | 1 | 10 ms |
| `vOLEDTask` | 256 | 1 | 50 ms（20 Hz） |
| `vVOFATask` | 256 | 1 | 5 ms（100 Hz） |

FreeRTOS：tick 1 kHz，heap_4，`configTOTAL_HEAP_SIZE` = 15 KB，`configMAX_PRIORITIES` = 16。

## FireWater 通道（8 路，100 Hz）

| 通道 | 含义 |
|---|---|
| 0 | 目标速度（rad/s） |
| 1 | 实际速度（rad/s，30 ms 低通后） |
| 2 / 3 / 4 | A / B / C 相电流（A） |
| 5 / 6 / 7 | A / B / C 相占空比（%） |

没有新鲜采样（未校准、或距上次 ADC 中断超过 10 ms）时，三相电流发送 0。

## 已知限制 / 待办

1. **电流只用于显示和有效性保护**，控制仍是电压模式（Uq 直接给电压矢量），没有电流闭环。
2. 板子无母线电流采样，`ADC_Current_Get()` 的 `bus_current` 恒为 0。
3. 共模扣除假设三相电流之和为 0（三线电机无中性线），带中性线或漏电时会引入偏差。
4. ADC 触发点是 TIM1_CH4 比较值 `ARR - 35`（≈ 4 us 采样 + 余量），改 PWM 频率/采样时间时要重新核对窗口。
5. 电角度零点存在 RAM 里，断电重启必须重新做 3 s 对齐。
6. 快照不含构建脚本和第三方库，不能独立编译；`.git` 不存在，版本只靠这些目录区分。
7. 工作区的 `build/` 目录属主不是当前 Windows 账户，在本机直接 `cmake --preset` 会因权限失败，
   需要先删除或重新授权该目录（本次快照未改动它）。
