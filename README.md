# ESP32-S3 + BNO085 IMU board: firmware and live dashboard

Firmware (ESP-IDF v6) for a Seeed XIAO ESP32-S3 carrier with a CEVA BNO085
9-axis IMU and a resistance front end (ADS1115 + analog mux, for flex
sensors), plus host tools: a CSV logger and a browser dashboard with live 3D
orientation, motion direction and inertial trajectories, over Bluetooth LE or
USB serial. Board files are in [`hardware/`](hardware/README.md).

以下为中文说明。

---

固件对应网表 `hardware/netlists/Netlist_ESP32-S3_XIAO_ADS1115_Resistance_Detector_7_2026-10-13.tel`，ESP-IDF v6.0.2。
下面的命令都在仓库根目录下运行。

## 烧录 & 看输出

```bash
source ~/esp/esp-idf/export.sh
idf.py -p /dev/cu.usbmodem101 flash monitor
```

端口名用 `ls /dev/cu.usbmodem*` 查。退出 monitor：`Ctrl+]`。
如果烧录时连不上：按住 XIAO 的 BOOT 键再插 USB，进下载模式。

日志同时输出到 UART0 和原生 USB，所以同一份固件在 XIAO（只有原生 USB）和
DevKitC 一类开发板（UART0 接 USB 转串口）上都能直接看。

### 用普通 ESP32-S3 开发板代替 XIAO

GPIO 编号和 XIAO 完全一样，固件不用改。XIAO 必须先从底板上拿下来，否则 3V3 会把它一起供电。

| 开发板 | 底板 | 网络 |
|---|---|---|
| 3V3 | J2.5 | 3V3 |
| GND | J2.6 | GND |
| GPIO1 | J1.1 | IMU_INT |
| GPIO2 | J1.2 | IMU_RST |
| GPIO5 | J1.5 | I2C_SDA |
| GPIO6 | J1.6 | I2C_SCL |

开发板走 USB 转串口时，端口名类似 `/dev/cu.usbserial-*`。

## 固件做了什么

| 步骤 | 内容 | 正常结果 |
|---|---|---|
| [1] | SDA/SCL/NRST/INT 各用内部下拉、上拉各读一次，判断高/低/浮空；再试 SDA/SCL 能否拉低、有没有短在一起 | 前三个 HIGH，INT 为 HIGH 或 LOW |
| [1b] | 用 ADC 测 SDA/SCL 实际电压（U1 运行 / 复位各一次），偏低时换算成对地漏电阻 | 都接近 3.1 V（ADC 上限） |
| [1c] | 只在总线电压偏低时运行：60 秒实时电压表，加热哪颗芯片电压就往下掉，用来定位漏电源 | — |
| [2] | 拉 NRST 复位，计时等 H_INTN 拉低 | 约 100 ms 内拉低 |
| [3] | I2C 全地址扫描（100 kHz） | `found 0x4A <- BNO085` |
| [4] | sh2 驱动读产品 ID，先 400 kHz，不行退到 100 kHz | 打印 part / sw 版本 |
| 流 | 输出 CSV 数据（26 列） | 每秒 50 行 |

[3] 没扫到时，会在 SDA 上听一下有没有 UART 数据（PS0/PS1 没读成 GND 时芯片会跑到 UART 模式），
然后打印诊断提示，每 5 秒重跑一遍。

`main.c` 里的 `ENABLE_TEST_CLOCK` 改成 1，会在 D3（J1.4）上输出精确的 32.768 kHz，
经 10 kΩ 飞线接到 XIN（C5 不接地端）可以绕过晶振排查。平时保持 0：它就在 SDA 旁边的排针上，会串扰 I2C。

## 数据格式

`#` 开头的是说明行，其他行是 CSV：

```
t_ms,qw,qx,qy,qz,yaw_deg,pitch_deg,roll_deg,ax,ay,az,gx,gy,gz,mx,my,mz,rv_status,mag_status,
gqw,gqx,gqy,gqz,lx,ly,lz
```

- `gq*`：Game Rotation Vector，不用磁力计，航向会慢慢漂，但不会因为附近有铁、电脑而跳
- `l*`：线性加速度（BNO085 自己去掉了重力），传感器坐标，m/s²

- 四元数 + 欧拉角（度），来自 Rotation Vector（融合磁力计）
- 加速度 m/s²（静止时模长约 9.8）、陀螺 rad/s（静止约 0）、磁场 µT（地磁约 25–65）
- `*_status`：0 不可靠 … 3 高精度。磁力计要在空中画几次 8 字才会升到 2–3
- 每 5 秒一行 `# rate Hz: ...`，应接近 rv/acc/gyro=100、mag=50

采样率和输出率在 `main/imu_stream.c` 顶部的 `REPORT_HZ` / `OUTPUT_HZ` 里改。

## 电脑端：存 CSV + 实时曲线

[host/imu_logger.py](host/imu_logger.py)，只依赖 pyserial 和 matplotlib（Homebrew 的 python3 已经装好）。

```bash
python3 host/imu_logger.py
```

- 自动找板子的串口（XIAO 原生 USB / FTDI / CP210x / CH34x），也可以用 `-p` 指定
- 数据存到 `host/logs/imu_<日期_时间>.csv`，`-o` 可改路径；第一列 `host_s` 是电脑收到该行的时间
- 窗口里四个面板：姿态角、加速度、角速度、磁场，右侧实时显示当前值，标题显示校准状态
- 关掉窗口就停止并保存；`--no-plot` 只记录（Ctrl+C 停止）
- `-w 20` 显示最近 20 秒；`--reset` 连接时复位板子，把启动过程也记下来
- 打开串口不会复位板子，可以随时接上去记录

固件每秒输出 50 行。要记录完整的 100 Hz 数据，把 `main/imu_stream.c` 里的 `OUTPUT_HZ` 改成 100。

串口控制台是 **921600** 波特率（`sdkconfig.defaults` 里用自定义 UART0 实现，默认 UART 的波特率改不了）。
两个脚本默认就是 921600；`idf.py monitor` 也会自动用 921600。XIAO 走原生 USB，与波特率无关。

## 演示界面：3D 姿态 + 运动方向（浏览器）

[host/imu_dashboard.py](host/imu_dashboard.py) + [host/web/index.html](host/web/index.html)，界面是英文的。
默认通过**蓝牙（BLE）**接收数据：板子广播名 `IMU-BNO085`，不用接串口线，只要板子有电。

```bash
python3 host/imu_dashboard.py
```

- 走串口线：加 `--serial`（自动找端口）或 `-p /dev/cu.xxx`
- 第一次用蓝牙时，macOS 会要求给终端 App 蓝牙权限（系统设置 → 隐私与安全性 → 蓝牙）
- 蓝牙包是 46 字节二进制（`main/ble_link.h` 的 `ble_sample_t`），电脑端还原成和串口一样的 26 列，
  所以录制、回放、日志脚本都通用；速率行、传感器重启、启动失败这些消息走另一个特征，也能在界面上看到

自动打开 http://localhost:8765 。3D 用 three.js，从 CDN 加载，**第一次打开需要联网**（之后浏览器有缓存）；
没网时 3D 区域会提示，数值、曲线、录制照常工作。

| 快捷键 | 作用 |
|---|---|
| T | 视图：姿态（板子固定在中心）↔ 轨迹（板子在空间里移动并画出轨迹） |
| G | 轨迹算法：手势（默认，连续画 8 字、画圈）↔ 笔画（走走停停，位置累积） |
| C | 清空轨迹，位置归零 |
| Z | 航向归零：把板子此刻的朝向定为"前"（拿板子指向屏幕再按）；轨迹也会清空 |
| S | 姿态源：平滑（不用磁力计，默认）↔ 指北（融合磁力计，先画 8 字校准） |
| M | 箭头含义：运动方向（加速度积分估算的速度）↔ 加速度 |
| R | 开始 / 停止录制 CSV（存到 `host/logs/`） |
| V | 视角复位（鼠标拖动可旋转、滚轮缩放） |
| F | 全屏 |

- 坐标：传感器输出东-北-天（ENU），已用重力和地磁实测核对；模型上 X 橙、Y 绿（板子前方）、Z 蓝
- 运动方向是估算：静止时自动归零，只看方向和快慢，不能当精确速度
- 手势算法：只积分最近 3 秒，每次积分后减掉一条最佳拟合直线（漂移就是这种趋势）。
  显示的是手势的形状和大小，自动转到正对屏幕；整只手的平移不累积。用实录的连续 8 字数据核对过，
  能稳定还原约 50–60 cm 宽的 "∞"；高通滤波法会把它扭成 U 形，所以没用
- 笔画算法：纯惯性推算（加速度积分两次）。误差随时间平方增长，所以按"笔画"算：每次停下时速度必为 0，
  用这一点把这段的漂移倒推扣掉（零速修正）。**每段移动前后各停 0.3 秒**最准；不停地晃，几秒后就会飘。
  模拟数据（含噪声和零偏）测试：走一个 40×30 cm 的方框加上下 25 cm，两圈后回到起点误差约 1.5 cm
- USB 被碰掉后会自动重连，不用重启程序
- 备用方案：`--replay host/logs/某次录制.csv` 循环回放录好的数据，板子出问题时也能演示

## 硬件文件

见 [hardware/README.md](hardware/README.md)：每个 Gerber 包和网表对应哪块板子，以及与当前实物板的差异。

## 第三方代码

`components/sh2/` 是 CEVA 的 sh2 驱动（[github.com/ceva-dsp/sh2](https://github.com/ceva-dsp/sh2)），
Apache License 2.0，原样保留源文件头和 `NOTICE.txt`，只加了 ESP-IDF 用的 `CMakeLists.txt`。

## 许可证

MIT，见 [LICENSE](LICENSE)。`components/sh2/` 除外，它沿用 CEVA 的 Apache License 2.0。
