| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- |

# BMD101

使用 UART 接口通信，1 个起始位，8 个数据位，1 个停止位，波特率位 57600

## 串口输出的数据包格式

| Header            | Data Payload | CRC    |
| ----------------- | ------------ | ------ |
| SYNC SYNC plength | payload[]    | chksum |

包括 Header（帧头）、data payload（数据有效载荷）、CRC 校验字节三个部分。

- SYNC 字节（其值均为 0xAA） 指示一帧数据的开始。
- plength 字节（0-169） 指示数据有效载荷部分的字节数。
- Data Payload 是由一系列的 DataRow 组成。

### DataRow 组成

| [EXCODE]... | CODE  | [vLength] | value[]         |
| ----------- | ----- | --------- | --------------- |
| ox55...     | 0-255 | 0-255     | Depends on CODE |

- DataRow 起始位可能有零个或多个[EXCODE]（扩展代码）字节，这些字节的值均为 0x55。
- EXCODE 字节数表示 Extended Code Level。
- Extended Code Level 是用来与[CODE]字节一起确定 DataRow 的数据表示的是什么方面的信息。

#### [CODE]字节

值在 0x00 和 0x7F 之间，那么 DataRow 就没有[LENGTH]字节，紧跟着[CODE]的是一个字节的[DATA]值，然后 DataRow 结束。

值在 0x80 和 0xFF 之间，那么紧跟着是[LENGTH]，它表示[DATA]的字节数。

#### 通常我们从 BMD101 接收到的 DataRow 主要有三种：

[CODE]=0x02，即信号质量数据，信号质量是一个介于 0-200 之间的数据，数值越大表示传感器采集到的信号质量越好，等于 0 时可能是因为电极与人体接触不良。

[CODE]=0x03，即实时的心率数据，用一个字节表示。这也是 BMD101 的方便之处，开发者不必再通过算法求出心率，只需通过读取串口发送的心率数据即可，心率数据一般每秒发送一次，但在信号质量不佳时仍然会输出心率数据，因此在使用心率数据前应该验证信号质量是否过差。

[CODE]=0x80 时，输出的数据[DATA]表示原始的心电波形数据，每一个数据由 16 位补码组成，其值范围为-32768 到 32767 之间。这 16 位数据的第一个字节是高 8 位字节，第二个字节是低 8 位，为了通过这两个字节还原心电波形数据值，可以通过以下代码完成：
short raw =(Value[0]<<8) | Value[1];

[DATA]可能包含了很多个心电数据，因此可能由很多个字节组成，通常 BMD101 每秒输出 512 个原始心电波形数据，即采样率为 512Hz。

## 数据包解析步骤

从数据流中连续读取字节，直到[SYNC]字节（0xAA）时。

读取下一个字节，并确保它也是一个[SYNC]字节（如果没有，返回步骤 1）。

读[PLENGTH]字节。

从[PAYLOAD…]中读取下一个 PLENGTH 字节，把它们保存在一个存储区域（如
unsigned char payload [ 256 ]数组）。按照接收的顺序累加每一个字节到校验器中。

使校验器中的低 8 位取反。这里是 C 代码：
checksum &= 0xff；
checksum = ~ checksum & 0xff；

读取[CRC]字节并验证它是否符合你的计算校验和（如果没有，返回到步骤 1）。

循环，直到 payload[]数组中所有字节（也就是 DataRows）被解析：
a）解析和计算[EXCODE]字节的数值（0x55），一般在当前 DataRow 的开始。 b）解析当前 DataRow 中的[CODE]字节数据。 c）如果适用，解析当前 DataRow 中的[LENGTH]字节数据。d）分析和处理 当前 DataRow 中的[DATA…]字节（或数组），基于 DataRow 的 [EXCODE]等级、[CODE]和[LENGTH]。 e）如果不是所有的字节都从 payload []数组中解析完成，返回 a）解析下一个 DataRow。

## ESP32 实现数据包解析

配置 ESP32 串口位中断接收方式，在中断服务程序中，完成数据包的接收与保存工作
