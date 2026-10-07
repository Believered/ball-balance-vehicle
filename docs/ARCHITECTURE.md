# 软件架构

## 入口与调度

`empty.c/main` 使用SysConfig初始化，先停机与初始化共享状态，再使能必要中断。主循环调用 `menu1` 选择页面，页面循环服务显示、按键和IMU。TIMA1每10ms更新轮速与控制，TIMG6每1ms执行按键消抖；位置环每20ms更新。

菜单编号与内部控制号 `jb` 不一致：当前主菜单1/2/3/4分别为第二问综合、速度调试、第四问底盘、第五问底盘，内部号8/1/9/10。其余主菜单见 [README](../README.md)。旧HRoute (`jb=3`)、旧位置调试 (`jb=4`) 和里程 (`jb=5`) 仍保留代码，旧界面部分被编译条件禁用。

## 模块

| 模块 | 职责 |
|---|---|
| `empty.syscfg` | 外设配置源；生成DriverLib初始化、启动/链接等配置 |
| `ENCODER` | A/B Gray合法跃迁、快照/清零、余数与历史速度标度、异常诊断 |
| `MOTER` | TB6612方向、PWM限幅、变向空档、制动与高阻释放 |
| `XUNJI/xunji` | 灰度预处理、位置误差、循迹与直线/定时任务、丢线/堵转检测 |
| `XUNJI/h_route` | HRoute历史路线、H2综合/标定、低冲击双轮启动与HBall底盘运输 |
| `PID` | 左右独立速度PI、位置PID、模式参数、限幅与抗饱和 |
| `MENU/KEY/OLED` | 页面与参数调整、1ms消抖、SSD1306显示 |
| `JY901/imu601` | IMU601 UART3帧解析、非阻塞服务、快照 |
| `USART1` | 有界UART格式化/解析；maxicam仅保留UART2接收链路 |

## 反馈与控制

Gray x4计数取幅值、除2并保留半步余数，10ms速度样本乘5维持原50ms参数尺度。坏反馈窗保护与模式相关，不能把它当作带符号位移。位置PID默认 `Kp=0.30, Ki=0.06, Kd=0.24`；速度左右参数独立。位置Kd使用轮速差阻尼，参数含义以 `PID/pid.c` 为准。

前进速度PI负输出归零，PWM最终0..1000，运输启动链包含同步爬行、逐轮确认和受限加速。`Control_ResetSpeedFeedback` 保存/恢复原中断屏蔽状态。NaN/Inf输入/参数归零并重置PI/PID状态。

## 异步输入边界

UART0默认RX关闭；启用旧CSV入口时须遵守CRLF、126字节内容上限与3整数格式。UART3接收IMU，UART2 maxicam收到换行置接收标志；当前控制器不消费球坐标，C2/C3 PWM保持零。

源码实现、测试结果和未完成能力见 [IMPLEMENTATION](IMPLEMENTATION.md)；硬件保护的条件和验证顺序见 [HARDWARE_SAFETY](../HARDWARE_SAFETY.md)。
