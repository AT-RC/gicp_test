# 串口通信协议分析 (virtual_serial_port)

## 1. 通信形式：USB-CDC (libusb)

与常见的 `/dev/ttyUSB0` 这种标准串口通信不同，该包使用的是基于 `libusb` 的 **USB-CDC (Communication Device Class)** 直接通信。

*   **非标准驱动**：不依赖 Linux 内核的 `usbserial` 驱动，而是直接通过 USB 厂商 ID (`VID: 0x0483`) 和产品 ID (`PID: 0x5740`) 锁定设备。
*   **异步传输**：使用了 `libusb_fill_bulk_transfer` 进行异步接收，并开启了独立线程进行高频发送（默认 100Hz）。
*   **支持热插拔**：代码中实现了 `libusb_hotplug_register_callback`，支持设备拔掉后再插入自动重连，这在机器人系统中非常稳定。

## 2. 发送内容：位姿数据 (PosePacket)

根据 `virtual_serial_port_node.cpp` 的定义，发送的是机器人在三维空间中的 **绝对位姿**：

*   **内容**：`x`, `y`, `z` 坐标（单位：米）以及 `yaw` 偏航角（单位：弧度）。
*   **来源**：通过 `tf2` 监听 `odom_frame` (如 `camera_init`) 到 `base_frame` (如 `aft_mapped`) 的变换，实时转换成浮点数发送。

### 协议格式（共 20 字节）

| 偏移  | 字段     | 类型       | 说明                                   |
| :---- | :------- | :--------- | :------------------------------------- |
| 0-1   | header   | uint8_t[2] | 固定为 `0x5A 0xA5`                     |
| 2-5   | x        | float      | X 轴位置                               |
| 6-9   | y        | float      | Y 轴位置                               |
| 10-13 | z        | float      | Z 轴位置                               |
| 14-17 | yaw      | float      | 偏航角                                 |
| 18    | checksum | uint8_t    | `x, y, z, yaw` 这 16 个字节的简单累加和 |
| 19    | tail     | uint8_t    | 固定为 `0xEE`                          |

## 3. 完整性评估

### 优点
*   **结构紧凑**：使用了 `#pragma pack(push, 1)` 确保没有内存对齐填充，字节序与大多数单片机（如 STM32）一致。
*   **有校验和**：包含基础的 `checksum`，可以过滤传输过程中的乱码。
*   **鲁棒性强**：在 `cdc_trans.cpp` 中有完善的状态机（`_disconnected`, `_need_reconnected`），能处理底层通信链路异常。

### 不完整/可改进之处
1.  **缺少下行处理**：`virtual_serial_port_node.cpp` 目前 **只发不收**。虽然 `CDCTrans` 类支持接收，但主节点里没有对下位机回传数据（如电量、编码器反馈、IMU）的处理逻辑。
2.  **校验强度较低**：`checksum` 是简单的 8 位累加，对于高频传输（100Hz）和长距离干扰，CRC16 或 CRC32 会更可靠。
3.  **缺乏版本握手**：没有在包头或包尾包含协议版本号。如果未来修改了 `PosePacket` 结构而下位机未更新，可能会导致解析错误甚至程序崩溃。
4.  **TF 异常处理**：在 `tf_thread_func` 中，如果 TF 丢失，程序只是捕获异常并等待，没有向单片机发送“定位失效”的状态位。

---

**总结**：对于将 SLAM 定位结果传给单片机做运动控制而言，这个通信方案是 **“可用且健壮”** 的，但在 **双向交互** 和 **数据安全** 上还有提升空间。
