EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr User 8465 5709
encoding utf-8
Sheet 1 1
Title "Balancing Robot Schematic"
Date "2020-10-12"
Rev ""
Comp "Kian Li Wan Po"
Comment1 "Department of Electrical Engineering"
Comment2 "University of Cape Town"
Comment3 ""
Comment4 ""
$EndDescr
Text Label 6850 1250 2    50   ~ 0
VDD
Wire Wire Line
	1950 4050 1700 4050
Text Label 1700 4050 0    50   ~ 0
NSS
Wire Wire Line
	1950 4150 1700 4150
Wire Wire Line
	1950 4250 1700 4250
Wire Wire Line
	1950 4350 1700 4350
Text Label 1700 4150 0    50   ~ 0
MOSI
Text Label 1700 4250 0    50   ~ 0
MISO
Text Label 1700 4350 0    50   ~ 0
SCLK
$Comp
L power:GND #PWR?
U 1 1 5F7D5882
P 6400 3700
F 0 "#PWR?" H 6400 3450 50  0001 C CNN
F 1 "GND" H 6405 3527 50  0000 C CNN
F 2 "" H 6400 3700 50  0001 C CNN
F 3 "" H 6400 3700 50  0001 C CNN
	1    6400 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F7937BA
P 2900 4950
F 0 "#PWR?" H 2900 4700 50  0001 C CNN
F 1 "GND" H 2905 4777 50  0000 C CNN
F 2 "" H 2900 4950 50  0001 C CNN
F 3 "" H 2900 4950 50  0001 C CNN
	1    2900 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 3700 6400 3600
Wire Wire Line
	6400 3600 6850 3600
NoConn ~ 7550 2800
NoConn ~ 7550 2900
Text Label 6950 1950 0    50   ~ 0
VDD
NoConn ~ 6750 2200
NoConn ~ 6150 3100
NoConn ~ 6150 3200
$Comp
L power:GND #PWR?
U 1 1 5F800501
P 6150 2800
F 0 "#PWR?" H 6150 2550 50  0001 C CNN
F 1 "GND" H 6155 2627 50  0000 C CNN
F 2 "" H 6150 2800 50  0001 C CNN
F 3 "" H 6150 2800 50  0001 C CNN
	1    6150 2800
	1    0    0    -1  
$EndComp
NoConn ~ 7550 3100
NoConn ~ 7550 3200
Text Label 7900 2600 2    50   ~ 0
Interrupt
$Comp
L Device:C C1
U 1 1 5F86A4F4
P 5050 1300
F 0 "C1" H 5165 1346 50  0000 L CNN
F 1 "10uF" H 5165 1255 50  0000 L CNN
F 2 "" H 5088 1150 50  0001 C CNN
F 3 "~" H 5050 1300 50  0001 C CNN
	1    5050 1300
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR?
U 1 1 5F89C8CE
P 4350 800
F 0 "#PWR?" H 4350 650 50  0001 C CNN
F 1 "+12V" H 4365 973 50  0000 C CNN
F 2 "" H 4350 800 50  0001 C CNN
F 3 "" H 4350 800 50  0001 C CNN
	1    4350 800 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F8AA3A2
P 5050 1450
F 0 "#PWR?" H 5050 1200 50  0001 C CNN
F 1 "GND" H 5055 1277 50  0000 C CNN
F 2 "" H 5050 1450 50  0001 C CNN
F 3 "" H 5050 1450 50  0001 C CNN
	1    5050 1450
	1    0    0    -1  
$EndComp
NoConn ~ 5650 1250
$Comp
L power:GND #PWR?
U 1 1 5F75F384
P 6150 1550
F 0 "#PWR?" H 6150 1300 50  0001 C CNN
F 1 "GND" H 6155 1377 50  0000 C CNN
F 2 "" H 6150 1550 50  0001 C CNN
F 3 "" H 6150 1550 50  0001 C CNN
	1    6150 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5F93AC27
P 1250 4300
F 0 "R2" V 1043 4300 50  0000 C CNN
F 1 "10k" V 1134 4300 50  0000 C CNN
F 2 "" V 1180 4300 50  0001 C CNN
F 3 "~" H 1250 4300 50  0001 C CNN
	1    1250 4300
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_MEC_5E SW2
U 1 1 5F94627C
P 1050 4050
F 0 "SW2" H 1050 4435 50  0000 C CNN
F 1 "SW_MEC_5E" H 1050 4344 50  0000 C CNN
F 2 "" H 1050 4350 50  0001 C CNN
F 3 "http://www.apem.com/int/index.php?controller=attachment&id_attachment=1371" H 1050 4350 50  0001 C CNN
	1    1050 4050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F97723B
P 1250 4550
F 0 "#PWR?" H 1250 4300 50  0001 C CNN
F 1 "GND" H 1255 4377 50  0000 C CNN
F 2 "" H 1250 4550 50  0001 C CNN
F 3 "" H 1250 4550 50  0001 C CNN
	1    1250 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 2600 7900 2600
NoConn ~ 6650 1050
$Comp
L Switch:SW_DPDT_x2 SW1
U 1 1 5F853194
P 4650 1050
F 0 "SW1" H 4650 1335 50  0000 C CNN
F 1 "SW_DPDT_x2" H 4650 1244 50  0000 C CNN
F 2 "" H 4650 1050 50  0001 C CNN
F 3 "~" H 4650 1050 50  0001 C CNN
	1    4650 1050
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F8861C4
P 4350 1450
F 0 "#PWR?" H 4350 1200 50  0001 C CNN
F 1 "GND" H 4355 1277 50  0000 C CNN
F 2 "" H 4350 1450 50  0001 C CNN
F 3 "" H 4350 1450 50  0001 C CNN
	1    4350 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 800  4350 950 
Wire Wire Line
	4350 950  4450 950 
Wire Wire Line
	4850 1050 5050 1050
Wire Wire Line
	5050 1050 5050 1150
Wire Wire Line
	4350 1450 4350 1150
Wire Wire Line
	4350 1150 4450 1150
Connection ~ 5050 1050
Text Label 5050 900  0    50   ~ 0
Vsupply
Wire Wire Line
	5050 1050 5650 1050
$Comp
L Regulator_Switching:LM2596S-3.3 U1
U 1 1 5F8323C6
P 6150 1150
F 0 "U1" H 6150 1517 50  0000 C CNN
F 1 "LM2596S-3.3" H 6150 1426 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TO-263-5_TabPin3" H 6200 900 50  0001 L CIN
F 3 "http://www.ti.com/lit/ds/symlink/lm2596.pdf" H 6150 1150 50  0001 C CNN
	1    6150 1150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5F86BF8E
P 6650 1400
F 0 "C2" H 6765 1446 50  0000 L CNN
F 1 "10uF" H 6765 1355 50  0000 L CNN
F 2 "" H 6688 1250 50  0001 C CNN
F 3 "~" H 6650 1400 50  0001 C CNN
	1    6650 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 1250 6850 1250
Connection ~ 6650 1250
Wire Wire Line
	6650 1550 6150 1550
Wire Wire Line
	6150 1550 6150 1450
Connection ~ 6150 1550
Wire Wire Line
	5050 900  5050 1050
Wire Wire Line
	5900 2700 6150 2700
Wire Wire Line
	5900 2600 6150 2600
Text Label 5900 2600 0    50   ~ 0
SDA
Text Label 5900 2700 0    50   ~ 0
SCL
Text Label 1600 3250 0    50   ~ 0
Interrupt
Wire Wire Line
	1950 3250 1600 3250
Wire Wire Line
	1250 4450 1250 4550
Wire Wire Line
	1250 4150 1250 4050
Wire Wire Line
	1250 3950 1250 4050
Connection ~ 1250 4050
Wire Wire Line
	1250 3950 1950 3950
Connection ~ 1250 3950
Wire Wire Line
	850  3950 850  4050
Text Label 750  3800 2    50   ~ 0
VDD
Wire Wire Line
	750  3950 750  3800
Wire Wire Line
	750  3950 850  3950
Connection ~ 850  3950
Text Label 1700 3850 0    50   ~ 0
IN1
Text Label 1700 3750 0    50   ~ 0
IN2
Wire Wire Line
	1700 3750 1950 3750
Wire Wire Line
	1700 3850 1950 3850
Text Label 1700 3550 0    50   ~ 0
IN3
Wire Wire Line
	1700 3550 1950 3550
Wire Wire Line
	1700 3450 1950 3450
Text Label 1700 3450 0    50   ~ 0
IN4
Text Label 1700 3650 0    50   ~ 0
PWM
Wire Wire Line
	1700 3650 1950 3650
NoConn ~ 2950 4350
NoConn ~ 2950 4250
NoConn ~ 2950 3950
NoConn ~ 2950 3850
NoConn ~ 2950 3750
NoConn ~ 2950 3650
NoConn ~ 2950 3450
NoConn ~ 2950 3150
NoConn ~ 2950 3050
NoConn ~ 2450 4650
NoConn ~ 2550 2650
NoConn ~ 2650 2650
NoConn ~ 2350 2650
NoConn ~ 1950 3050
NoConn ~ 1950 3150
Wire Wire Line
	2900 4650 2900 4950
Text Label 3200 4150 2    50   ~ 0
SCL
$Comp
L Motor:Motor_DC M?
U 1 1 5F7DF0A8
P 3250 1100
F 0 "M?" H 3408 1096 50  0000 L CNN
F 1 "Motor_DC" H 3408 1005 50  0000 L CNN
F 2 "" H 3250 1010 50  0001 C CNN
F 3 "~" H 3250 1010 50  0001 C CNN
	1    3250 1100
	1    0    0    -1  
$EndComp
$Comp
L Motor:Motor_DC M?
U 1 1 5F7E0723
P 3250 1800
F 0 "M?" H 3408 1796 50  0000 L CNN
F 1 "Motor_DC" H 3408 1705 50  0000 L CNN
F 2 "" H 3250 1710 50  0001 C CNN
F 3 "~" H 3250 1710 50  0001 C CNN
	1    3250 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 2100 2950 2100
Wire Wire Line
	2950 2100 2950 1700
Wire Wire Line
	2950 1300 2950 900 
Wire Wire Line
	2950 900  3250 900 
Text Label 1050 1400 0    50   ~ 0
IN3
Text Label 1050 1500 0    50   ~ 0
IN4
Wire Wire Line
	2950 1700 2500 1700
Wire Wire Line
	2500 1400 3250 1400
$Comp
L Driver_Motor:L298HN U2
U 1 1 5F7D2F0C
P 1900 1500
F 0 "U2" H 1900 2381 50  0000 C CNN
F 1 "L298HN" H 1900 2290 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-15_P2.54x2.54mm_StaggerOdd_Lead4.58mm_Vertical" H 1950 850 50  0001 L CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/CD00000240.pdf" H 2050 1750 50  0001 C CNN
	1    1900 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 1600 3250 1600
Wire Wire Line
	2500 1300 2950 1300
Text Label 2200 800  0    50   ~ 0
Vsupply
Text Label 1050 1000 0    50   ~ 0
IN1
Text Label 1050 1100 0    50   ~ 0
IN2
Wire Wire Line
	1050 1500 1300 1500
Wire Wire Line
	1050 1400 1300 1400
Wire Wire Line
	1050 1100 1300 1100
Wire Wire Line
	1050 1000 1300 1000
Wire Wire Line
	900  1200 900  1600
Wire Wire Line
	900  1600 1300 1600
Wire Wire Line
	900  1200 1300 1200
Text Label 700  1200 0    50   ~ 0
PWM
Wire Wire Line
	700  1200 900  1200
NoConn ~ 1900 800 
Connection ~ 900  1200
NoConn ~ 1600 2200
NoConn ~ 1700 2200
$Comp
L power:GND #PWR?
U 1 1 5F7D40FA
P 1900 2200
F 0 "#PWR?" H 1900 1950 50  0001 C CNN
F 1 "GND" H 1905 2027 50  0000 C CNN
F 2 "" H 1900 2200 50  0001 C CNN
F 3 "" H 1900 2200 50  0001 C CNN
	1    1900 2200
	1    0    0    -1  
$EndComp
Text Label 5650 2750 2    50   ~ 0
MISO
Text Label 5650 2600 2    50   ~ 0
SCLK
Text Label 5650 2900 2    50   ~ 0
MOSI
Wire Wire Line
	4100 3350 3850 3350
Wire Wire Line
	3850 2900 4100 2900
Wire Wire Line
	3850 2750 4100 2750
Text Label 3850 2750 0    50   ~ 0
SDA
Text Label 3850 2900 0    50   ~ 0
SCL
Text Label 3850 3350 0    50   ~ 0
NSS
Text Label 5650 3500 2    50   ~ 0
VDD
$Comp
L power:GND #PWR?
U 1 1 5F78CDD9
P 3850 3700
F 0 "#PWR?" H 3850 3450 50  0001 C CNN
F 1 "GND" H 3855 3527 50  0000 C CNN
F 2 "" H 3850 3700 50  0001 C CNN
F 3 "" H 3850 3700 50  0001 C CNN
	1    3850 3700
	1    0    0    -1  
$EndComp
Text Label 3800 2150 0    50   ~ 0
VDD
NoConn ~ 5400 3650
$Comp
L Device:C C4
U 1 1 5F8686F2
P 3500 3000
F 0 "C4" H 3615 3046 50  0000 L CNN
F 1 "0.1uF" H 3615 2955 50  0000 L CNN
F 2 "" H 3538 2850 50  0001 C CNN
F 3 "~" H 3500 3000 50  0001 C CNN
	1    3500 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 2300 3850 2150
Wire Wire Line
	3850 2300 4100 2300
Wire Wire Line
	3500 2850 3500 2300
Wire Wire Line
	3500 2300 3850 2300
Connection ~ 3850 2300
Wire Wire Line
	3500 3150 3500 3650
Wire Wire Line
	3500 3650 3850 3650
Wire Wire Line
	3850 3650 3850 3700
Connection ~ 3850 3650
Wire Wire Line
	3850 3650 4100 3650
Text Label 3200 4050 2    50   ~ 0
SDA
NoConn ~ 5400 2300
NoConn ~ 5400 2450
NoConn ~ 5400 3050
NoConn ~ 5400 3200
NoConn ~ 5400 3350
Wire Wire Line
	2000 800  2200 800 
Wire Wire Line
	3200 4150 2950 4150
Wire Wire Line
	3200 4050 2950 4050
Wire Wire Line
	2550 4650 2900 4650
$Comp
L MCU_Module:Arduino_Nano_v3.x A1
U 1 1 5F9B6DA0
P 2450 3650
F 0 "A1" H 2450 2561 50  0000 C CNN
F 1 "Arduino_Nano_v3.x" H 2450 2470 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 2450 3650 50  0001 C CIN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 2450 3650 50  0001 C CNN
	1    2450 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 1950 6950 2200
$Comp
L Sensor_Motion:MPU-6050 U4
U 1 1 5F7CFBB8
P 6850 2900
F 0 "U4" H 6850 2111 50  0000 C CNN
F 1 "MPU-6050" H 6850 2020 50  0000 C CNN
F 2 "Sensor_Motion:InvenSense_QFN-24_4x4mm_P0.5mm" H 6850 2100 50  0001 C CNN
F 3 "https://store.invensense.com/datasheets/invensense/MPU-6050_DataSheet_V3%204.pdf" H 6850 2750 50  0001 C CNN
	1    6850 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 5FAA5465
P 1000 3350
F 0 "D1" H 993 3567 50  0000 C CNN
F 1 "LED" H 993 3476 50  0000 C CNN
F 2 "" H 1000 3350 50  0001 C CNN
F 3 "~" H 1000 3350 50  0001 C CNN
	1    1000 3350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5FAA7208
P 1400 3350
F 0 "R1" V 1193 3350 50  0000 C CNN
F 1 "100" V 1284 3350 50  0000 C CNN
F 2 "" V 1330 3350 50  0001 C CNN
F 3 "~" H 1400 3350 50  0001 C CNN
	1    1400 3350
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FAACB41
P 700 3350
F 0 "#PWR?" H 700 3100 50  0001 C CNN
F 1 "GND" H 705 3177 50  0000 C CNN
F 2 "" H 700 3350 50  0001 C CNN
F 3 "" H 700 3350 50  0001 C CNN
	1    700  3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	700  3350 850  3350
Wire Wire Line
	1150 3350 1250 3350
Wire Wire Line
	1550 3350 1950 3350
Wire Wire Line
	5650 3500 5400 3500
Wire Wire Line
	5400 2900 5650 2900
Wire Wire Line
	5400 2750 5650 2750
Wire Wire Line
	5400 2600 5650 2600
$Comp
L Flow~Deck~v2:FlowDeck U3
U 1 1 5F74C86D
P 4750 3000
F 0 "U3" H 4750 3925 50  0000 C CNN
F 1 "FlowDeck" H 4750 3834 50  0000 C CNN
F 2 "" H 4750 2150 50  0001 C CNN
F 3 "" H 4750 2150 50  0001 C CNN
	1    4750 3000
	1    0    0    -1  
$EndComp
NoConn ~ 4100 3200
NoConn ~ 4100 3050
NoConn ~ 4100 3500
NoConn ~ 4100 2600
NoConn ~ 4100 2450
$EndSCHEMATC
