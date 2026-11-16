# ODrive USB 串口控制

支持力矩、速度和位置三种运行模式。通过板载 USB CDC 虚拟 COM 口连接。ODriveFOC 1.2 的「串口控制」窗口可以选轴、切换模式、下发目标和读取反馈。通用串口终端也可直接发送 ASCII 命令。[Windows USB 串口驱动说明](https://learn.microsoft.com/windows-hardware/drivers/usbcon/usb-driver-installation-based-on-compatible-ids)

## 连接和启动

1. 连接待调板的 USB 数据口，选择固件枚举出的 COM 口。终端设为 115200、8 数据位、无校验、1 停止位、无硬件流控。接收 CRLF，发送 CR、LF 或 CRLF。发送 `info` 可看见 `protocol=2`。
2. 发送 `prepare`，查询 `status`，等 `offset=1`。此时电流零偏已完成采集。对所选轴执行 `cal 0` 或 `cal 1`，校准会带动转子。
3. 校准和运行期间，串口工具可以每 250 ms 自动发一条 `ping`。板端心跳超时为 2 s。PC 控制窗口在收到自己发出的运动命令确认后自动发心跳。查询 `feedback 0`，等 `cal=1` 后再发送运行目标。
4. `mode 0 speed` 选择速度模式并使 M0 置空闲。再发送 `set 0 1` 可下发 1 rad/s。直接发送 `speed 0 1` 也可完成选模式和下发目标。

## 三种模式

| 模式 | 一条命令 | 目标含义 |
| --- | --- | --- |
| 力矩 | `torque 0 0.001` | M0 的目标为 +0.001 N·m，按 `Iq=目标力矩/Kt` 转换并限流 |
| 速度 | `speed 0 1` | M0 的机械速度目标为 +1 rad/s |
| 位置 | `position 0 0.1` | M0 相对机械零点的多圈位置目标为 +0.1 rad |

`mode <轴> <torque|speed|position>` 关闭该轴 PWM、清空目标和控制状态。`set <轴> <目标>` 使用已选择的模式。直接命令切到不同模式时会重置该轴控制状态。同一模式的连续目标更新保留积分和斜坡。兼容命令 `velocity <轴> <rad/s>` 等同于 `speed`，`current <轴> <A>` 使用力矩模式并按每轴 Kt 换算。

**Kt 预设值为 0.0315 N·m/A，来自仿真电机参数。** `kt 0 0.05` 可以为空闲轴设置实测 Kt，设置保存在 RAM，重启或 `clear` 恢复默认。`limits 0` 返回当前三种模式的目标限制。`feedback` 的 `torque_est_nm` 根据测得的 Iq 计算。[PMSM 电磁转矩关系](https://www.mathworks.com/help/mcb/ref/pmsmtorqueestimator.html)

## 命令索引

| 指令 | 用途 |
| --- | --- |
| `info` / `help` | 查询协议、版本和命令表 |
| `status` / `feedback <轴>` | 查看整机诊断，或查看单轴模式、位置、速度、Iq、估算力矩和故障 |
| `mode <轴>` / `limits <轴>` | 查询所选模式和当前限制 |
| `prepare` / `cal <轴>` | 初始化功率级和零偏，校准所选电机 |
| `zero <轴>` | 将已校准、空闲轴的当前位置设为机械零点 |
| `kt <轴> <N·m/A>` / `tune <轴> <Kp> <Ki>` | 修改空闲轴的 Kt 或速度 PI 增益 |
| `idle <轴>` / `stop` | 关闭单轴 PWM，或关闭两轴及公共功率使能 |
| `ping` / `clear` | 维持心跳，或复位故障及校准状态 |
| `encoder` / `scan <轴>` / `trace <轴> [页]` / `dfu` | 编码器诊断、I²C 扫描、轨迹与 DFU |

单条命令上限为 95 个 ASCII 字符。成功回包以 `OK` 开始，`ping` 返回 `pong`。查询 `status` 与 `encoder` 保留原诊断格式。错误回包以 `ERR` 开始。`feedback`、`status` 和 `limits` 只读取状态，运行需心跳或持续更新目标。主机停止读取时固件优先保持接收和停机指令通畅，可能丢弃无法完整写入发送缓冲的回包。

源码运行 `python tools/install_test_deps.py` 后，可用 `python tools/control_gui.py` 或 `python tools/console.py COM7`。请将 `COM7` 换成对应端口。
