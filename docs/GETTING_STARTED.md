# 环境搭建与编译烧录

## 软件与版本

从工程记录恢复：CCS 70.5.0（Theia）、MSPM0 SDK 2.10.0.04、SysConfig 1.26.2、TI Clang 4.0.4.LTS。不使用RTOS。这里只记录工程配置，当前电脑未安装完整TI工具链，尚未执行整套编译；不保证最新版或Classic CCS直接兼容。

从TI官方获得工具及SDK，在CCS产品设置中关联SDK，确保 `${COM_TI_MSPM0_SDK_INSTALL_DIR}` 可解析。若版本不可获得，按TI迁移文档建立新工程并迁入业务源码，逐项检查外设/链接/编译选项，不能只换版本字符串。

## 导入与构建

1. 打开CCS的 Import Project，选择本仓库根目录，导入工程 `motor`。
2. 检查 `.cproject` 的SDK、SysConfig和TI Clang产品路径。
3. 打开 `empty.syscfg`，确认 MSPM0G3507、UART、定时器PWM及GPIO映射与实物一致。不要手工编辑生成的 `ti_msp_dl_config.h/.c`。
4. Build。SysConfig/SDK会生成或提供配置头/源、启动和链接/编译选项等文件。`Debug/` 属于生成产物，不入库；生成配置不成功时先解决版本和产品路径。
5. 阅读编译与链接日志，确认没有未解决符号和无效选项。当前仓库的桌面测试不替代此步骤。

参考：[TI SysConfig 文件生成说明](https://software-dl.ti.com/msp430/esd/MSPM0-SDK/2_08_00_03/docs/english/tools/sysconfig_guide/doc_guide/doc_guide-srcs/sysconfig_guide.html)。该链接解释生成机制，目标工程仍以记录的2.10.0.04配置为基准。

## 硬件与运行

需要MSPM0G3507板、XDS-110、TB6612与电机/编码器、8路红外、I2C OLED（0x3C）、IMU601和4按键。Key1..4当前配置为PB11/PB0/PB7/PB10；完整接线以 `empty.syscfg` 为准。自制板不能只照搬LaunchPad跳线。

先阅读 [硬件安全说明](../HARDWARE_SAFETY.md)，断开电机验证逻辑供电，再通过 `targetConfigs/MSPM0G3507.ccxml` 下载。LaunchPad SWD跳线须按开发板手册配置。调试断点会暂停控制环，不能在高速运行时用断点验证实时性。

启动显示 `Ready` 后进入8项菜单：K1下一项，K2确认。各运行页通过RUN/EXIT选择启动或退出；故障后第一次确认可能只清故障，需要再明确启动一次。不要把历史 `HRoute` 的内部 `jb` 与菜单编号混用，见 [实施说明](IMPLEMENTATION.md)。

## 桌面验证

Linux/WSL安装GCC后在仓库根目录：

```bash
bash tests/run_host_tests.sh
```

两个测试分别验证有界串口解析和真实PID纯逻辑，使用ASan/UBSan；测试用 `tests/stubs` 不包含真实DriverLib/外设模拟，不能用于烧录。详细边界见 [IMPLEMENTATION.md](IMPLEMENTATION.md)。
