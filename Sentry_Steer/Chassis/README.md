# 平衡步兵

## PIN MAP——Chassis

(* 表示没用到)

### CAN1

PA11

PA12

- 0x1FF: 500Hz-
- 0x206: 500Hz-
- 0x100: 500Hz-
- 0x101 : 500Hz-
- 0x151 : 500Hz-

### CAN2 

PB12

PB13

- 
- 

### USART2 (PC-COM )*

PA2

PA3

### USART1 & USART6 (A1) 

PA9

PA10

&

PC6

PC7

### IIC  (INA260)

SCL PB10

SDA PB11

### GPIO (LED)

Red  PC9

Blue PC8

### ADC1

PA4

### USART4 (Referee)

PC10

PC11

### 超级电容

电容充放电

PA6 

PA7 

电池供电

PC5

电容供电

PC4

## PIN MAP——Gimbal

### USART1 (DJI Remote)

RX PA10

### CAN2 

PB12

PB13

### CAN1

PB8

PB9

- 0x206 YAW
- 0x160 Chassis

### USART2 (PC)

PA2

PA3

### IIC*

PB6

PB7

### TIM3 (Servo)

PB1





## 跳跃

### 原地跳跃

腿长增加之后重心位置改变，导致失去平衡：

- 空中能否调节平衡？（关闭原先的控制器）
- 着陆后是否能增大恢复平衡的速度？

空中轮子快速旋转，导致通过编码器解算的YAW出现较大偏差，落地后会快速自旋。

如何协调position对平衡的补偿以及应对外部干扰的调节

起跳全过程的扭矩输出



- 跳跃后前后移动（向电池盒方向移动）
- 跑跳失衡
- （但是现在正反向都能不摔跤了）
- 输出还是没拉满？（三角形的）

- 跳跃能力不稳定
