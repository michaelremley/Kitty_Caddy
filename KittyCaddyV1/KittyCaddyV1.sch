EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Kitty Caddy"
Date "2021-10-15"
Rev "1.0"
Comp "Michael Remley and Co."
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector:Barrel_Jack_MountingPin J1
U 1 1 6145CEAB
P 2000 1750
F 0 "J1" H 2057 2067 50  0000 C CNN
F 1 "20V Supply" H 2057 1976 50  0000 C CNN
F 2 "@KittyCaddyLib:BarrelJack_CUI_PJ-102AH_Horizontal" H 2050 1710 50  0001 C CNN
F 3 "~" H 2050 1710 50  0001 C CNN
	1    2000 1750
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR06
U 1 1 61480D9F
P 2700 1450
F 0 "#PWR06" H 2700 1300 50  0001 C CNN
F 1 "+12V" H 2715 1623 50  0000 C CNN
F 2 "" H 2700 1450 50  0001 C CNN
F 3 "" H 2700 1450 50  0001 C CNN
	1    2700 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 1850 2400 1850
$Comp
L power:GND #PWR013
U 1 1 614ADE4A
P 8475 2550
F 0 "#PWR013" H 8475 2300 50  0001 C CNN
F 1 "GND" H 8480 2377 50  0000 C CNN
F 2 "" H 8475 2550 50  0001 C CNN
F 3 "" H 8475 2550 50  0001 C CNN
	1    8475 2550
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW3
U 1 1 614BEF71
P 8725 2350
F 0 "SW3" H 8725 2635 50  0000 C CNN
F 1 "UP" H 8725 2544 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H9.5mm" H 8725 2550 50  0001 C CNN
F 3 "~" H 8725 2550 50  0001 C CNN
	1    8725 2350
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW2
U 1 1 614C27CF
P 8725 1950
F 0 "SW2" H 8725 2235 50  0000 C CNN
F 1 "MID" H 8725 2144 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H9.5mm" H 8725 2150 50  0001 C CNN
F 3 "~" H 8725 2150 50  0001 C CNN
	1    8725 1950
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 614C2BF7
P 8725 1550
F 0 "SW1" H 8725 1835 50  0000 C CNN
F 1 "DOWN" H 8725 1744 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H9.5mm" H 8725 1750 50  0001 C CNN
F 3 "~" H 8725 1750 50  0001 C CNN
	1    8725 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	9175 1550 9175 1700
Text GLabel 7925 1575 2    50   Input ~ 0
SDA
Text GLabel 7925 1475 2    50   Input ~ 0
SCL
Text GLabel 7575 4775 0    50   Input ~ 0
SCL
Text GLabel 7575 4875 0    50   Input ~ 0
SDA
$Comp
L power:+5V #PWR01
U 1 1 614B2CEE
P 7625 1475
F 0 "#PWR01" H 7625 1325 50  0001 C CNN
F 1 "+5V" H 7640 1648 50  0000 C CNN
F 2 "" H 7625 1475 50  0001 C CNN
F 3 "" H 7625 1475 50  0001 C CNN
	1    7625 1475
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 614B3766
P 7075 1675
F 0 "#PWR04" H 7075 1425 50  0001 C CNN
F 1 "GND" H 7080 1502 50  0000 C CNN
F 2 "" H 7075 1675 50  0001 C CNN
F 3 "" H 7075 1675 50  0001 C CNN
	1    7075 1675
	1    0    0    -1  
$EndComp
$Comp
L KittyCaddyLib:OLED_SSD1306 DS1
U 1 1 614BE660
P 7675 2175
F 0 "DS1" H 7225 2575 50  0000 C CNN
F 1 "OLED_SSD1306" H 7675 1825 50  0000 C CNN
F 2 "@KittyCaddyLib:SSD1306" H 7675 1775 50  0001 C CNN
F 3 "~" H 7675 2175 50  0001 C CNN
	1    7675 2175
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C3
U 1 1 6148ED80
P 2950 1750
F 0 "C3" H 3065 1796 50  0000 L CNN
F 1 "100uF" H 3065 1705 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 2950 1750 50  0001 C CNN
F 3 "~" H 2950 1750 50  0001 C CNN
	1    2950 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8925 1550 9175 1550
Wire Wire Line
	8525 1550 8475 1550
Wire Wire Line
	8475 1550 8475 1950
Wire Wire Line
	8525 1950 8475 1950
Connection ~ 8475 1950
Wire Wire Line
	8475 1950 8475 2350
Wire Wire Line
	8525 2350 8475 2350
Connection ~ 8475 2350
Wire Wire Line
	8475 2350 8475 2550
Wire Wire Line
	2300 1650 2400 1650
Wire Wire Line
	2400 1650 2400 1600
Wire Wire Line
	2000 2050 2400 2050
Wire Wire Line
	2400 1850 2400 1900
$Comp
L power:GND #PWR011
U 1 1 614BCAD2
P 2400 2150
F 0 "#PWR011" H 2400 1900 50  0001 C CNN
F 1 "GND" H 2405 1977 50  0000 C CNN
F 2 "" H 2400 2150 50  0001 C CNN
F 3 "" H 2400 2150 50  0001 C CNN
	1    2400 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 2050 2400 2150
Connection ~ 2400 2050
Wire Wire Line
	9075 1800 9075 1950
Wire Wire Line
	9075 1950 8925 1950
Wire Wire Line
	8925 2350 9175 2350
Wire Wire Line
	9175 2350 9175 1900
$Comp
L Device:C C2
U 1 1 614FAB57
P 2500 1750
F 0 "C2" H 2615 1796 50  0000 L CNN
F 1 "0.1uF" H 2615 1705 50  0000 L CNN
F 2 "Capacitor_THT:C_Rect_L4.6mm_W2.0mm_P2.50mm_MKS02_FKP02" H 2538 1600 50  0001 C CNN
F 3 "~" H 2500 1750 50  0001 C CNN
	1    2500 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 1900 2400 1900
Connection ~ 2400 1900
Wire Wire Line
	2400 1900 2400 2050
Wire Wire Line
	2500 1600 2400 1600
Wire Wire Line
	7075 1675 7525 1675
Wire Wire Line
	7825 1675 7825 1575
Wire Wire Line
	7825 1575 7925 1575
Wire Wire Line
	7725 1675 7725 1475
Wire Wire Line
	7725 1475 7925 1475
Wire Wire Line
	7625 1675 7625 1475
Wire Wire Line
	2500 1600 2700 1600
Connection ~ 2500 1600
Wire Wire Line
	2950 1900 2500 1900
Connection ~ 2500 1900
Text GLabel 9175 1700 2    50   Input ~ 0
SW_DOWN
Text GLabel 9175 1800 2    50   Input ~ 0
SW_MID
Text GLabel 9175 1900 2    50   Input ~ 0
SW_UP
Wire Wire Line
	9075 1800 9175 1800
$Comp
L power:GND #PWR017
U 1 1 6146A997
P 5700 4700
F 0 "#PWR017" H 5700 4450 50  0001 C CNN
F 1 "GND" H 5705 4527 50  0000 C CNN
F 2 "" H 5700 4700 50  0001 C CNN
F 3 "" H 5700 4700 50  0001 C CNN
	1    5700 4700
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C4
U 1 1 6145FEE6
P 5700 4550
F 0 "C4" H 5815 4596 50  0000 L CNN
F 1 "100uF" H 5815 4505 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 5700 4550 50  0001 C CNN
F 3 "~" H 5700 4550 50  0001 C CNN
	1    5700 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 4400 5150 4400
Wire Wire Line
	5150 4300 5150 4400
$Comp
L power:+12V #PWR015
U 1 1 61482B41
P 5150 4300
F 0 "#PWR015" H 5150 4150 50  0001 C CNN
F 1 "+12V" H 5165 4473 50  0000 C CNN
F 2 "" H 5150 4300 50  0001 C CNN
F 3 "" H 5150 4300 50  0001 C CNN
	1    5150 4300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR021
U 1 1 61483BDB
P 5050 6050
F 0 "#PWR021" H 5050 5800 50  0001 C CNN
F 1 "GND" H 5055 5877 50  0000 C CNN
F 2 "" H 5050 6050 50  0001 C CNN
F 3 "" H 5050 6050 50  0001 C CNN
	1    5050 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 6050 5050 5950
Wire Wire Line
	5050 5950 5150 5950
Connection ~ 5050 5950
Wire Wire Line
	4950 5950 5050 5950
Connection ~ 5150 4400
Wire Wire Line
	5150 4400 5150 4450
Text GLabel 4550 4750 0    50   Input ~ 0
STEP_RESET
Text GLabel 4550 4850 0    50   Input ~ 0
STEP_SLEEP
Text GLabel 4550 5050 0    50   Input ~ 0
STEP_EN
Text GLabel 4500 5650 0    50   Input ~ 0
MS3
Text GLabel 4500 5550 0    50   Input ~ 0
MS2
Text GLabel 4500 5450 0    50   Input ~ 0
MS1
Wire Wire Line
	4550 5650 4500 5650
Wire Wire Line
	4550 5550 4500 5550
Wire Wire Line
	4550 5450 4500 5450
Text GLabel 4550 5250 0    50   Input ~ 0
DIR
Text GLabel 4550 5150 0    50   Input ~ 0
STEP
Wire Wire Line
	5450 5350 5550 5350
Wire Wire Line
	5450 5250 5550 5250
Wire Wire Line
	5450 5150 5550 5150
Wire Wire Line
	5450 5050 5550 5050
$Comp
L Connector_Generic:Conn_01x04 J3
U 1 1 61458863
P 5750 5150
F 0 "J3" H 5830 5142 50  0000 L CNN
F 1 "Stepper Motor" H 5830 5051 50  0000 L CNN
F 2 "Connector_Molex:Molex_KK-254_AE-6410-04A_1x04_P2.54mm_Vertical" H 5750 5150 50  0001 C CNN
F 3 "~" H 5750 5150 50  0001 C CNN
	1    5750 5150
	1    0    0    -1  
$EndComp
$Comp
L Driver_Motor:Pololu_Breakout_A4988 A2
U 1 1 6145BA9C
P 4950 5150
F 0 "A2" H 4700 5800 50  0000 C CNN
F 1 "Pololu_Breakout_A4988" H 5800 4650 50  0001 C CNN
F 2 "@KittyCaddyLib:Pololu_Breakout-16_15.2x20.3mm" H 5225 4400 50  0001 L CNN
F 3 "https://www.pololu.com/product/2980/pictures" H 5050 4850 50  0001 C CNN
	1    4950 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 4450 4950 4300
$Comp
L power:+5V #PWR014
U 1 1 61481E76
P 4950 4300
F 0 "#PWR014" H 4950 4150 50  0001 C CNN
F 1 "+5V" H 4965 4473 50  0000 C CNN
F 2 "" H 4950 4300 50  0001 C CNN
F 3 "" H 4950 4300 50  0001 C CNN
	1    4950 4300
	1    0    0    -1  
$EndComp
$Comp
L KittyCaddyLib:Module_DS3231_RTC A3
U 1 1 615A85E2
P 7775 4925
F 0 "A3" H 7775 5250 50  0000 C CNN
F 1 "Module_DS3231_RTC" H 7675 4500 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x08_P2.54mm_Vertical" H 8075 4125 50  0001 C CNN
F 3 "~" H 7775 4925 50  0001 C CNN
	1    7775 4925
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR08
U 1 1 615A9EB4
P 8075 4525
F 0 "#PWR08" H 8075 4375 50  0001 C CNN
F 1 "+5V" H 8090 4698 50  0000 C CNN
F 2 "" H 8075 4525 50  0001 C CNN
F 3 "" H 8075 4525 50  0001 C CNN
	1    8075 4525
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 615AA501
P 8175 5425
F 0 "#PWR010" H 8175 5175 50  0001 C CNN
F 1 "GND" H 8180 5252 50  0000 C CNN
F 2 "" H 8175 5425 50  0001 C CNN
F 3 "" H 8175 5425 50  0001 C CNN
	1    8175 5425
	1    0    0    -1  
$EndComp
Text GLabel 8825 5075 2    50   Input ~ 0
ALARM
NoConn ~ 8825 4825
NoConn ~ 8175 4525
NoConn ~ 7575 5125
Wire Notes Line
	6700 2850 9800 2850
Wire Notes Line
	9800 2850 9800 1200
Wire Notes Line
	9800 1200 6700 1200
Wire Notes Line
	6700 1200 6700 2850
Text Notes 6750 1300 0    50   ~ 0
User Interface\n
Wire Notes Line
	7100 4300 7100 5800
Wire Notes Line
	7100 5800 9200 5800
Wire Notes Line
	9200 5800 9200 4300
Wire Notes Line
	9200 4300 7100 4300
Text Notes 7150 4400 0    50   ~ 0
Real Time Clock
Text Notes 7300 3050 0    50   ~ 0
Optical Homing Sensor
Wire Notes Line
	9150 2950 7250 2950
Wire Notes Line
	9150 4100 9150 2950
Wire Notes Line
	7250 4100 9150 4100
Wire Notes Line
	7250 2950 7250 4100
Connection ~ 7800 3925
Wire Wire Line
	7800 3925 8375 3925
Wire Wire Line
	8275 3725 8275 3625
Connection ~ 8275 3725
Wire Wire Line
	8275 3725 8250 3725
Wire Wire Line
	8275 3825 8275 3725
Wire Wire Line
	8375 3825 8275 3825
Wire Wire Line
	8275 3375 8275 3525
Wire Wire Line
	8375 3525 8275 3525
Wire Wire Line
	7800 3925 7800 3825
Wire Wire Line
	7700 3925 7800 3925
Wire Wire Line
	8275 3625 8375 3625
$Comp
L Device:R_Small_US R3
U 1 1 614A59EB
P 8275 3275
F 0 "R3" H 8207 3229 50  0000 R CNN
F 1 "220" H 8207 3320 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" H 8275 3275 50  0001 C CNN
F 3 "~" H 8275 3275 50  0001 C CNN
	1    8275 3275
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small_US R4
U 1 1 614D86F8
P 7800 3725
F 0 "R4" H 7732 3679 50  0000 R CNN
F 1 "50k" H 7732 3770 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" H 7800 3725 50  0001 C CNN
F 3 "~" H 7800 3725 50  0001 C CNN
	1    7800 3725
	-1   0    0    1   
$EndComp
Text GLabel 7700 3925 0    50   Input ~ 0
HOMING
$Comp
L power:+5V #PWR019
U 1 1 6147BA13
P 7800 3625
F 0 "#PWR019" H 7800 3475 50  0001 C CNN
F 1 "+5V" H 7815 3798 50  0000 C CNN
F 2 "" H 7800 3625 50  0001 C CNN
F 3 "" H 7800 3625 50  0001 C CNN
	1    7800 3625
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR020
U 1 1 614798AE
P 8250 3725
F 0 "#PWR020" H 8250 3475 50  0001 C CNN
F 1 "GND" H 8255 3552 50  0000 C CNN
F 2 "" H 8250 3725 50  0001 C CNN
F 3 "" H 8250 3725 50  0001 C CNN
	1    8250 3725
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR018
U 1 1 614789FD
P 8275 3175
F 0 "#PWR018" H 8275 3025 50  0001 C CNN
F 1 "+5V" H 8290 3348 50  0000 C CNN
F 2 "" H 8275 3175 50  0001 C CNN
F 3 "" H 8275 3175 50  0001 C CNN
	1    8275 3175
	1    0    0    -1  
$EndComp
$Comp
L KittyCaddyLib:OPB740_Reflective_Object_Sensor J4
U 1 1 6147700E
P 8625 3725
F 0 "J4" H 8525 4075 50  0000 C CNN
F 1 "OPB740_Reflective_Object_Sensor" H 8625 3725 50  0001 C CNN
F 2 "Connector_Molex:Molex_KK-254_AE-6410-05A_1x05_P2.54mm_Vertical" H 8625 3725 50  0001 C CNN
F 3 "" H 8625 3725 50  0001 C CNN
	1    8625 3725
	1    0    0    -1  
$EndComp
Wire Notes Line
	3900 4050 6400 4050
Wire Notes Line
	6400 4050 6400 6300
Wire Notes Line
	6400 6300 3900 6300
Wire Notes Line
	3900 6300 3900 4050
Text Notes 3950 4150 0    50   ~ 0
Stepper Driver
Wire Notes Line
	1700 1200 3350 1200
Wire Notes Line
	3350 1200 3350 2400
Wire Notes Line
	3350 2400 1700 2400
Wire Notes Line
	1700 2400 1700 1200
Text Notes 1750 1300 0    50   ~ 0
DC Barrel Power
Wire Wire Line
	2700 1450 2700 1600
Connection ~ 2700 1600
Wire Wire Line
	2700 1600 2950 1600
NoConn ~ 4250 2750
Text Notes 4975 1225 0    50   ~ 0
*use Arduino 5V DC-DC 
Text Notes 3675 1225 0    50   ~ 0
Microcontroller
Wire Notes Line
	6475 3675 3625 3675
Wire Notes Line
	3625 1125 3625 3675
Wire Notes Line
	6475 1125 3625 1125
Wire Notes Line
	6475 3675 6475 1125
Text GLabel 5250 2450 2    50   Input ~ 0
ALARM
Text GLabel 4250 2650 0    50   Input ~ 0
STEP_RESET
Text GLabel 4250 2550 0    50   Input ~ 0
STEP_SLEEP
Text GLabel 4250 1950 0    50   Input ~ 0
STEP_EN
Text GLabel 5250 2550 2    50   Input ~ 0
SW_UP
Text GLabel 5250 2650 2    50   Input ~ 0
SW_MID
Text GLabel 5250 2750 2    50   Input ~ 0
SW_DOWN
$Comp
L power:+12V #PWR02
U 1 1 614B097A
P 4650 1450
F 0 "#PWR02" H 4650 1300 50  0001 C CNN
F 1 "+12V" H 4665 1623 50  0000 C CNN
F 2 "" H 4650 1450 50  0001 C CNN
F 3 "" H 4650 1450 50  0001 C CNN
	1    4650 1450
	1    0    0    -1  
$EndComp
Connection ~ 4950 1450
Wire Wire Line
	5550 1450 4950 1450
Wire Wire Line
	5550 1500 5550 1450
Text GLabel 4250 2250 0    50   Input ~ 0
MS3
Text GLabel 4250 2150 0    50   Input ~ 0
MS2
Text GLabel 4250 2050 0    50   Input ~ 0
MS1
$Comp
L MCU_Module:Arduino_Nano_Every A1
U 1 1 6145AA09
P 4750 2450
F 0 "A1" H 4750 2450 50  0000 C CNN
F 1 "Arduino_Nano_Every" H 4250 1500 50  0000 C CNN
F 2 "@KittyCaddyLib:Arduino_Nano" H 4750 2450 50  0001 C CIN
F 3 "https://content.arduino.cc/assets/NANOEveryV3.0_sch.pdf" H 4750 2450 50  0001 C CNN
	1    4750 2450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 614B5F14
P 5550 1800
F 0 "#PWR05" H 5550 1550 50  0001 C CNN
F 1 "GND" H 5555 1627 50  0000 C CNN
F 2 "" H 5550 1800 50  0001 C CNN
F 3 "" H 5550 1800 50  0001 C CNN
	1    5550 1800
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 614B3970
P 5550 1650
F 0 "C1" H 5665 1696 50  0000 L CNN
F 1 "0.1uF" H 5665 1605 50  0000 L CNN
F 2 "Capacitor_THT:C_Rect_L4.6mm_W2.0mm_P2.50mm_MKS02_FKP02" H 5588 1500 50  0001 C CNN
F 3 "~" H 5550 1650 50  0001 C CNN
	1    5550 1650
	1    0    0    -1  
$EndComp
NoConn ~ 5250 1950
NoConn ~ 5250 1850
NoConn ~ 4850 1450
NoConn ~ 4250 1650
NoConn ~ 4250 1750
NoConn ~ 4250 2950
NoConn ~ 4250 3050
NoConn ~ 4250 3150
NoConn ~ 5250 3050
Connection ~ 6050 2950
Wire Wire Line
	6050 2950 6100 2950
Wire Wire Line
	6050 2900 6050 2950
Connection ~ 5750 2850
Wire Wire Line
	5750 2850 5800 2850
Wire Wire Line
	5750 2800 5750 2850
Wire Wire Line
	5250 2950 6050 2950
Text GLabel 6100 2950 2    50   Input ~ 0
SCL
Text GLabel 5800 2850 2    50   Input ~ 0
SDA
$Comp
L power:GND #PWR016
U 1 1 614BE7E1
P 4750 3450
F 0 "#PWR016" H 4750 3200 50  0001 C CNN
F 1 "GND" H 4755 3277 50  0000 C CNN
F 2 "" H 4750 3450 50  0001 C CNN
F 3 "" H 4750 3450 50  0001 C CNN
	1    4750 3450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR09
U 1 1 61494483
P 5750 2600
F 0 "#PWR09" H 5750 2450 50  0001 C CNN
F 1 "+5V" H 5765 2773 50  0000 C CNN
F 2 "" H 5750 2600 50  0001 C CNN
F 3 "" H 5750 2600 50  0001 C CNN
	1    5750 2600
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR012
U 1 1 61493CA8
P 6050 2700
F 0 "#PWR012" H 6050 2550 50  0001 C CNN
F 1 "+5V" H 6065 2873 50  0000 C CNN
F 2 "" H 6050 2700 50  0001 C CNN
F 3 "" H 6050 2700 50  0001 C CNN
	1    6050 2700
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R2
U 1 1 614933DC
P 6050 2800
F 0 "R2" H 5982 2754 50  0000 R CNN
F 1 "10k" H 5982 2845 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" H 6050 2800 50  0001 C CNN
F 3 "~" H 6050 2800 50  0001 C CNN
	1    6050 2800
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small_US R1
U 1 1 6149262B
P 5750 2700
F 0 "R1" H 5682 2654 50  0000 R CNN
F 1 "10k" H 5682 2745 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" H 5750 2700 50  0001 C CNN
F 3 "~" H 5750 2700 50  0001 C CNN
	1    5750 2700
	-1   0    0    1   
$EndComp
Wire Wire Line
	5250 2850 5750 2850
Text GLabel 5250 3150 2    50   Input ~ 0
HOMING
Text GLabel 4250 2450 0    50   Input ~ 0
DIR
Text GLabel 4250 2350 0    50   Input ~ 0
STEP
$Comp
L power:+5V #PWR03
U 1 1 6147EE6E
P 4950 1450
F 0 "#PWR03" H 4950 1300 50  0001 C CNN
F 1 "+5V" H 4965 1623 50  0000 C CNN
F 2 "" H 4950 1450 50  0001 C CNN
F 3 "" H 4950 1450 50  0001 C CNN
	1    4950 1450
	1    0    0    -1  
$EndComp
NoConn ~ 5250 2250
$EndSCHEMATC
