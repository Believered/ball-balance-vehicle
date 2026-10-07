# 车载平衡滚球控制：验证记录

日期：2026-10-07。以下为匿名整理副本的本轮检查，历史图表和原材料结果另行标注。

## 环境与复查入口

WSL / GCC，C11；ASan 与 UBSan。

```text
bash tests/run_host_tests.sh
```

## 结果

解析器与 PID 两组主机测试通过：容量边界、CRLF、嵌入 NUL、溢出恢复、PID 限幅及积分边界。

## 验证边界

未编译完整 TI 工程、未烧录或验证实车成绩。CCS、SDK、板级参数见上手文档。

源码及安装说明见 [仓库 README](../README.md)，许可范围见 [RIGHTS.md](../RIGHTS.md)。
