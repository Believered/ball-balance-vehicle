# 车载滚球运输底盘控制（MSPM0G3507）

基于 TI MSPM0G3507 的电机、正交编码器、8路红外循迹、IMU 与 OLED 控制工程，提供低冲击启动、速度/位置环、路线运输与故障停机逻辑。材料以 H 题任务组织，比赛年份和规则以原始赛题为准。

当前实现以**底盘运动控制**为核心。K230 UART2 链路保留接收接口，球坐标识别和托盘舵机闭环尚未实现。本次验证了桌面可测试的串口/PID逻辑；整套固件编译、实车运输效果和赛题指标仍需指定工具链与硬件验证。详情见 [源码实施说明](docs/IMPLEMENTATION.md)。

## 功能与技术栈

- TIMA1 每10ms采集轮速并运行控制；8路灰度位置环以20ms运行。
- 完整 Gray 正交计数折算为历史速度标度，左右独立 PI、PWM限幅/斜率约束、低冲击双轮启动。
- 菜单显示、按键消抖、IMU非阻塞解析、路线阶段状态和堵转/超速/反馈异常停机。
- UART解析限制帧长度，拒绝整数溢出和格式错误；PID对非有限输入归零并允许后续恢复。
- C、TI DriverLib/SysConfig、Code Composer Studio；不使用 RTOS。

## 安装与运行

工程文件记录的版本为 **CCS 70.5.0（Theia）、MSPM0 SDK 2.10.0.04、SysConfig 1.26.2、TI Clang 4.0.4.LTS**。先按版本恢复环境，再逐项迁移；没有验证其他版本或 Classic CCS 的兼容性。

1. 安装上述 TI 工具，确保 CCS 识别 MSPM0 SDK 产品路径。
2. 用 CCS 导入根目录的 `.project` / `.cproject` / `.ccsproject`（工程名 `motor`）。
3. 打开 `empty.syscfg` 检查 MSPM0G3507 和板级接线，执行 Build。配置头/源、启动和链接配置由 SysConfig/SDK 管理，生成目录不入库。
4. 阅读 [上手指南](docs/GETTING_STARTED.md) 和 [硬件安全说明](HARDWARE_SAFETY.md)，先断开电机验证逻辑部分，再限流、架空测试。
5. 通过 `targetConfigs/MSPM0G3507.ccxml` 的 XDS-110 下载。启动 OLED 到 `Ready` 后，K1选下一项，K2进入/确认；运行页的启停含义以页面提示为准。

桌面纯逻辑验证可在 Linux/WSL（GCC）运行，不需要开发板：

```bash
bash tests/run_host_tests.sh
```

测试使用 ASan/UBSan，只编译共享解析器和真实 `PID/pid.c`。`tests/stubs` 不能当作固件 DriverLib 替代品。

## 使用示例与菜单

| 主菜单 | 显示 | 当前实现入口 |
|---|---|---|
| 1 | H2 Final | `menu2` / H2Final综合路线，内部 `jb=8` |
| 2 | Speed PID | `menu3` / 速度环调试，内部 `jb=1` |
| 3 | H4 Ball AB | `menu4` / 第四问底盘运输，内部 `jb=9` |
| 4 | H5 Ball Lap | `menu5` / 第五问底盘运输，内部 `jb=10` |
| 5 | H5 PID Lap | `menu6` / 位置PID调参路线，内部 `jb=2` |
| 6 | Straight | `menu7` / 直线/距离任务，内部 `jb=2` |
| 7 | Timed Track | `menu8` / 定时循迹，内部 `jb=2` |
| 8 | H2 Curve | `menu9` / 第二问弯道标定，内部 `jb=7` |

菜单号与 `jb` 是不同概念。旧 `HRoute`（`jb=3`）和位置调试等代码保留作对照，原菜单部分处于 `#if 0`，不代表当前可直接从主菜单启动。

## K230接口与供电

无线视频由 K230 独立采集、编码、发送，不经过 MSPM0。预留低带宽串口为 K230 GPIO11/UART2_TXD → MSPM0 PB18/UART2_RX、GPIO12/UART2_RXD ← PA23/UART2_TX，共地，9600波特率。K230端需先完成 FPIOA 引脚映射；串口有接收缓冲并不代表完成坐标识别与滚球闭环。

电机与逻辑供电需按实物电路核对。软件没有电流传感器，故障逻辑不能替代硬件过流保护；默认标定参数也不等于另一台车的合格参数。

## 目录与文档

```text
empty.c / empty.syscfg          入口与外设配置
board.c / board.h              板级辅助（保留LCKFB来源）
ENCODER/ MOTER/ XUNJI/ PID/     反馈、电机、路线、控制器
MENU/ KEY/ OLED/ JY901/        菜单、按键、显示、姿态
USART1/                        串口辅助、有界解析器、K230预留链路
targetConfigs/                 调试目标
tests/                         桌面C边界测试
docs/                          环境、架构、源码实施说明
artifacts/report_revision/media/ 历史说明图（示意，非实测结果）
```

- [上手指南](docs/GETTING_STARTED.md)、[架构](docs/ARCHITECTURE.md)、[实施说明与验证](docs/IMPLEMENTATION.md)
- [循迹与调参](CONTROL_GUIDE.md)、[低冲击运输控制](BALL_CONTROL_GUIDE.md)、[历史HRoute指南](H_TASK3_GUIDE.md)
- [安全与故障处理](HARDWARE_SAFETY.md)、[第三方来源](THIRD_PARTY_NOTICES.md)

## 许可与来源

许可见仓库许可文件。TI模板/DriverLib和LCKFB板级代码保留各自来源及版权，见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。本仓库没有分发 CCS/SDK/编译器；不要把工具链或第三方材料重新标为项目原创。

[本轮验证记录](docs/VALIDATION.md)（2026-10-07）。
