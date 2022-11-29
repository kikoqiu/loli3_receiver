# loli3_receiver
loli3 arduino receiver with ibus sbus support
arduino pro mini

arduino pro mini loli3 遥控器接收
支持ibus输出，串口rx接pin 3。用vJoySerialFeeder可以连电脑模拟器 https://github.com/Cleric-K/vJoySerialFeeder
支持sbus输出，pin 2 （与标协议有差别，每秒只有50个数据，未测试是否可用），遥控器5通道启用sbus输出 configure ch5 to sbus in controller to enable sbus output。
pin4 pin5 短接重新对码


 * NRF24L01 | Arduino
 * CE    -> 7
 * CSN   -> 8
 * MOSI_pin  -> 11
 * MISO_pin  -> 12
 * SCK_pin   -> 13
 * IRQ   -> 
 * VCC   -> < 3.6V
 * GND   -> GND

 * ibus   --> 3
 * sbus   --> 2
 * vcc voltage meter --> a3
 * vbat voltage meter --> a2 (加电位器 add Potentiometer)

