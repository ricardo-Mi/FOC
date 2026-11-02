# V5 速度 + 电流双闭环 FOC（快照）

- 快照时间：2026-11-02
- 对应代码：工作区 `src/`、`inc/` 中当前参与编译的应用文件
- 文件数：29 个源文件/头文件 + 本 README
- 定位：在 V4 速度环 + 电流采样（电压模式）的基础上，加入 **29.8 kHz dq 电流环**，
  速度环输出 `iq*`，电流环输出 `ud/uq`，形成速度 + 电流双闭环

本目录只用于留档，不参与编译。完整工程（Drivers、FreeRTOS、CMake、linker 等）
保留在本地，本快照不能单独构建。

## 与 V4 的主要差异

| 项目 | V4_闭环速度+电流采样 | V5_速度电流闭环 |
|---|---|---|
| 控制结构 | 速度环 PID 直接输出 `Uq`（电压模式） | 速度环 PI 输出 `iq*`，dq 电流环 29.8 kHz 输出 `ud/uq` |
| 新增文件 | — | `FOC_Current.c/h`、`FOC_Params.h` |
| 电流环 | 无 | d/q PI + 电压矢量限幅 + 积分抗饱和 |
| 速度环参数 | `FOC_VEL_KP/KI` 直接写电压 | `J=2e-5`、`Beta=12`，按 `Kpw=Beta*J/Kt` 推导 |
| 编码器方向 | `AS5600_DIRECTION = -1` | **`AS5600_DIRECTION = 1`（本板实测）** |
| 电流符号 | `I=(offset-raw)*scale` | 本板实测 `I=(raw-offset)*scale` |
| 共模处理 | 直接赋值 | 20 ms 一阶低通后再扣除 |
| 过流阈值 | 6 A（临时诊断值） | 2 A |
| VOFA | 8 通道，921600 | 10 通道，921600，新增 `iq_ref`、`iq_meas` |
| 停机路径 | 已验证 | 已验证，MOE 关闭、DRV8323 使能拉低 |

## 文件清单

| 文件 | 作用 |
|---|---|
| `main.c` | 系统初始化、任务创建、FreeRTOS 钩子 |
| `FOC.c` / `FOC.h` | 状态机、速度环、PWM、电角度对齐 |
| `FOC_Current.c` / `FOC_Current.h` | dq 电流环、反 Park/Clarke、零序注入、过流保护 |
| `FOC_Params.h` | 电机参数、电流环/速度环参数、诊断开关 |
| `ADC.c` / `ADC.h` | ADC1/2/3 三相同步采样、零点校准、共模低通 |
| `AS5600.c` / `AS5600.h` | AS5600 软件 I2C、角度、差分测速 |
| `Key.c` / `Key.h` | PC0 启停按键 |
| `OLED.c` / `OLED.h` / `OLED_Font.h` | 电流、状态、故障显示 |
| `VOFA.c` / `VOFA.h` | FireWater 10 通道格式化 + 100 Hz 任务 |
| `VOFA_UART.c` / `VOFA_UART.h` | USART3 PD8 发送，921600 |
| `Board.c` / `Board.h` | PA2 DRV8323 故障输入 |
| `Delay.c` / `Delay.h` | DWT 微秒计时/延时 |
| `LowPassFilter.c` / `LowPassFilter.h` | 一阶低通滤波 |
| `PID.c` / `PID.h` | 旧 PID 实现；当前 FOC 不再使用，保留作历史参考 |
| `FreeRTOSConfig.h` | FreeRTOS 配置 |
| `stm32f4xx_conf.h` | 标准外设库配置 |

## 引脚与外设

| 功能 | 引脚 | 说明 |
|---|---|---|
| A 相 PWM | PE9 / PE8 | TIM1_CH1 / CH1N |
| B 相 PWM | PE11 / PE10 | TIM1_CH2 / CH2N |
| C 相 PWM | PA10 / PE12 | TIM1_CH3 / CH3N |
| PWM 频率 | — | 168 MHz / 22 / 256 ≈ 29.8 kHz，死区 0.5 µs |
| DRV8323 使能 | PE7 | 高有效 |
| DRV8323 CAL | PE5 | 高电平约 200 µs 触发内部失调校准 |
| A/B/C 相电流 | PA4 / PA5 / PA3 | ADC1_IN4 / ADC2_IN5 / ADC3_IN3，注入组同步 |
| 编码器 | PB13 SCL / PB12 SDA | AS5600，7 位地址 0x36，方向 1 |
| 启停按键 | PC0 | 上拉输入，低有效 |
| 故障输入 | PA2 | DRV8323 nFAULT，上拉输入 |
| OLED | PF9 / PF10 | 软件 I2C |
| 遥测 | PD8 | USART3 TX，921600 8N1，FireWater |

## 关键参数

```text
Pn       = 7              (12N14P)
R        = 5.1 ohm
Ld = Lq  = 1 mH
PSI_F    = 0.00358 Wb     (KV=220)
J        = 2.0e-5 kg*m^2
VDC      = 12.6 V
Alpha    = 200 rad/s      (电流环闭环极点)
Beta     = 12 rad/s       (速度环闭环极点)
VEL_LPF_Tf = 0.030 s
Iq_Max   = 1.0 A
V_Max    = 0.72*VDC/sqrt(3) ≈ 5.24 V
OC_Limit = 2.0 A
ALIGN_Vol = 3.0 V
ALIGN_Time = 3000 ms
ALIGN_Angle = 1.5*pi      (ud=0, uq=ALIGN_Vol；实际合成矢量在 0°)
```

## 启动状态机

```text
STOP -> WAKE(300ms) -> CAL(DRV8323内部校准) -> OFST(PWM开启后零点复校)
     -> ALGN(3s电角度对齐) -> RUN
```

- 第一次按 PC0 启动会执行约 3 秒电角度对齐，电机必须空载或可自由转动。
- 校准完成后 `foc_calibrated=1`，同一供电周期内再次启停会跳过 ALGN。
- 断电重启会重新校准。
- ERROR 状态下按一次 PC0 清除故障，再按一次重新启动。

## 电流环与速度环

- 速度环：1 kHz，输入 `target_velocity - measured_velocity`，输出 `iq*`。
- 电流环：29.8 kHz（ADC 注入中断内），`id*=0`，PI 输出 `ud/uq`。
- 反 Park / 反 Clarke + 最小-最大零序注入生成三路占空比。
- 电压矢量限幅 `V_Max`，电流限幅 `Iq_Max`，积分抗饱和 `Kaw_Cur/Kaw_Spd`。
- 电角度：`theta_e = Pn * AS5600_GetMechanicalAngle() - zero_offset`；
  1 kHz 用编码器做相位修正，30 kHz 用 `we_est` 外推。

## VOFA 通道（10 路，100 Hz，921600）

| 通道 | 含义 |
|---|---|
| 0 | 目标速度（rad/s） |
| 1 | 实际速度（rad/s，30 ms 低通后） |
| 2 / 3 / 4 | A / B / C 相电流（A） |
| 5 / 6 / 7 | A / B / C 相 PWM 占空比（%） |
| 8 | iq_ref（速度环输出，A） |
| 9 | iq_meas（电流环实际 q 轴电流，A） |

- 通道 8/9 用于观察电流环跟踪：RUN 时 `iq_meas` 应紧贴 `iq_ref`。
- STOP/ALIGN/OFST 时通道 8/9 为 0。
- VOFA 是 100 Hz 采样，只能看电流环低频包络和跟踪误差；
  30 kHz PWM 纹波和 5 ms 级快速阶跃要用示波器看 SOx（PA4/PA5/PA3）。

## 实测结果

- 目标 15 rad/s，空载：实测约 **14.98 rad/s**，连续转 10 圈以上。
- 停机测试：RUN 5 秒后自动停机，状态回到 STOP，MOE 关闭、DRV8323 使能拉低。
- 正常连续运行和停机路径未出现 HardFault。
- 低速 5 rad/s 存在约 ±2 rad/s 的低频摆动，属于低速齿槽/摩擦 + 30 ms 滤波；
  如需低速平稳，可把 `Beta` 降到 8~10 或增大 `VEL_LPF_Tf`。

## 已知限制 / 待办

1. AS5600 驱动未检查 I2C ACK，传感器掉线时角度可能读到 0xFFF，闭环会静默失效。
2. DRV8323 nFAULT 目前只在 OLED 显示，未参与 PWM 关断保护。
3. ADC 触发点到更新事件余量约 0.14 µs；改 PWM 频率/采样时间时需重新核对窗口，
   可考虑把采样时间从 84 cycles 改为 56 cycles。
4. 未做 ADC 内部自校准，只做了 DRV8323 两次零点校准。
5. 速度环低速（约 5 rad/s）有轻微 hunting，15 rad/s 稳态良好。
6. 快照不包含 Drivers/FreeRTOS/CMake/linker，不能单独构建。
7. `src/`、`inc/` 被仓库 `.gitignore` 忽略；完整工程请本地备份。

## 使用说明

- 完整工程中按 `CMakeLists.txt` 的 `APP_SOURCES` 编译。
- 烧录：`build/gcc-debug/stm32f407vgt6_app.elf`。
- VOFA：921600，8N1，FireWater，10 通道。
- 如果电机能对齐但闭环不转或转向相反，把 `src/AS5600.c` 的
  `AS5600_DIRECTION` 从 `1` 改成 `-1`，或交换电机任意两相线。
