# 相机模式使用与排障手册

本 fork 原创功能：给 Muse 语音固件加上一套完整的板载相机（GC0308）——按键进入、取景、拍照存 SD 卡、照片回聊天识别、远程拍照带小图。官方与上游移植版均无此功能。

英文版：[camera-mode.en.md](camera-mode.en.md) · 返回[仓库首页](../../README.md)

---

## 给 AI 助手的说明（先读这一节）

如果你是被用户叫来帮忙构建、烧录或排查这套固件的 AI 助手（Agent），请先读完这一节，再从第 0 步开始。

### 目标

让板子跑上带相机模式的固件，并陪用户完成验收：双击进相机、拍一张、退出后照片回到聊天里被认出来。

### 执行约定

- **构建与烧录是两个独立动作**：`tools/muse/board.sh build szp` 和 `board.sh flash szp` 分开执行；`flash` 只烧构建目录里的现成产物、从不编译。改完代码必须先 `build` 看到成功收尾，再 `flash`——直接 flash 烧回的是旧固件。
- 看构建进度只用一条：`tail -f /tmp/muse_build_szp.log`。给用户的每条构建命令都要附带这一条。
- 给用户的命令必须是最终可执行值，不留占位符让用户自己替换。
- 手把手时一次只给一步，等用户贴出输出确认后再给下一步。
- 烧录前核对刷机包 md5（构建机、传输中转、本机三处一致才烧）。

### 必须先停下来、得到用户明确同意才能做的操作

- 向板子烧录固件（任何版本，包括回滚）。
- 把 SD 卡拔出读照片（照片的唯一原件在卡上，拔卡前先确认没有正在写入）。
- 把任何凭据（SDK token、WiFi 密码）写进仓库文件或聊天记录。

### 需要用户本人动手的操作

- 插拔 USB、按语音键（双击 / 短按 / 长按）、举拍摄物做验收。
- 在自己的账号上登录、输入密码与 token、合并 GitHub PR。

### 绝对不能自作主张做的事

- 相机不应答时直接拔线断电——先按第 3 节留证据，再断电。
- 用旧构建产物冒充新构建烧录；构建失败时反复重试同一条命令而不看日志。
- 在公开仓库里提交任何真实凭据——`devices/sdkconfig.muse-lichuang-szp` 中的 token 类配置项在提交前必须置空。

---

## 0. 准备

- **板子**：立创实战派 ESP32-S3 N16R8，GC0308 摄像头接 J1 座；SD 卡 FAT32，照片目录 `/DCIM`。
- **固件**：本仓库 `main`（相机模式自 cammode 系列起并入），用 `szp` 预设构建。
- **电脑**：ESP-IDF v6.0.1 环境；串口 `/dev/ttyUSB0`（CH340）。串口监视器接入会经 EN 复位板子一次，属正常现象、不是崩溃。

## 1. 构建与烧录

### 1.1 构建

在 `esp32/` 目录下：

```bash
tools/muse/board.sh build szp
tail -f /tmp/muse_build_szp.log
```

成功收尾的标志是日志出现 `Project build complete`。

### 1.2 打包刷机包（便于中转与回滚）

构建产物目录 `build-muse-lichuang-szp/` 里取六件，按此布局打包并记下 tar 包 md5：

```text
bootloader/bootloader.bin
partition_table/partition-table.bin
ota_data_initial.bin
srmodels/srmodels.bin
muse-gadget.bin
flash_args
```

### 1.3 烧录（先退串口监视器；刷前留回滚包）

串口监视器占用 `/dev/ttyUSB0`，烧录前先退出（监视器里按 `Ctrl+]`）。在刷机包解开的目录里、IDF 环境已 source 的前提下：

```bash
python -m esptool --chip esp32s3 -p /dev/ttyUSB0 -b 460800 --before default-reset --after hard-reset write-flash @flash_args
```

回滚包 = 上一个定版的刷机包，刷法相同。**先留回滚包，再刷
...[truncated 4072 chars]