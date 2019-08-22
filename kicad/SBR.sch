EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:motor_drivers
LIBS:Msystem
LIBS:dc-dc
LIBS:arduino
LIBS:SBR-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Sel Balancing Robot"
Date "8/7/2019"
Rev "V1.0"
Comp "Raspibo"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L L293 U?
U 1 1 5D1F3BF5
P 6900 5100
F 0 "U?" H 6700 6125 50  0000 R CNN
F 1 "L293" H 6700 6050 50  0000 R CNN
F 2 "Housings_DIP:DIP-16_W7.62mm" H 7150 4350 50  0001 L CNN
F 3 "" H 6600 5800 50  0001 C CNN
	1    6900 5100
	1    0    0    1   
$EndComp
$Comp
L 74HC04 U?
U 1 1 5D1F3CE3
P 5950 4700
F 0 "U?" H 6100 4800 50  0000 C CNN
F 1 "74HC04" H 6150 4600 50  0000 C CNN
F 2 "" H 5950 4700 50  0001 C CNN
F 3 "" H 5950 4700 50  0001 C CNN
	1    5950 4700
	1    0    0    -1  
$EndComp
$Comp
L 74HC04 U?
U 2 1 5D1F3D67
P 5950 5300
F 0 "U?" H 6100 5400 50  0000 C CNN
F 1 "74HC04" H 6150 5200 50  0000 C CNN
F 2 "" H 5950 5300 50  0001 C CNN
F 3 "" H 5950 5300 50  0001 C CNN
	2    5950 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 4900 5500 4900
Wire Wire Line
	5500 4900 5500 4700
Wire Wire Line
	6400 5500 5500 5500
Wire Wire Line
	5500 5500 5500 5300
$Comp
L Arduino_Nano_Socket XA?
U 1 1 5D1F43E5
P 5400 2250
F 0 "XA?" V 5500 2250 60  0000 C CNN
F 1 "Arduino_Nano_Socket" V 5300 2250 60  0000 C CNN
F 2 "" H 7200 6000 60  0001 C CNN
F 3 "" H 7200 6000 60  0001 C CNN
	1    5400 2250
	1    0    0    -1  
$EndComp
$Comp
L MPU-6050 U?
U 1 1 5D1F4833
P 1450 2100
F 0 "U?" H 1150 2700 60  0000 L CNN
F 1 "MPU-6050" V 1650 1850 60  0000 L CNN
F 2 "" H 6450 -700 60  0001 C CNN
F 3 "" H 6450 -700 60  0001 C CNN
	1    1450 2100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1900 1950 4100 1950
Wire Wire Line
	1900 2050 4100 2050
Text GLabel 7250 1450 2    60   Input ~ 0
PWM1
Text GLabel 7200 2250 2    60   Input ~ 0
PWM2
Wire Wire Line
	6900 1550 6700 1550
Text GLabel 5500 4700 0    60   Input ~ 0
PWM1
Text GLabel 5500 5300 0    60   Input ~ 0
PWM2
Text GLabel 6400 5100 0    60   Input ~ 0
EN
Text GLabel 6400 5700 0    60   Input ~ 0
EN
Text GLabel 6900 1950 2    60   Input ~ 0
EN
Text GLabel 1900 2550 2    60   Input ~ 0
IntMPU
$Comp
L GND #PWR?
U 1 1 5D1F5F89
P 1900 1800
F 0 "#PWR?" H 1900 1550 50  0001 C CNN
F 1 "GND" H 1900 1650 50  0000 C CNN
F 2 "" H 1900 1800 50  0001 C CNN
F 3 "" H 1900 1800 50  0001 C CNN
	1    1900 1800
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5D1F62DB
P 3950 2750
F 0 "#PWR?" H 3950 2500 50  0001 C CNN
F 1 "GND" H 3950 2600 50  0000 C CNN
F 2 "" H 3950 2750 50  0001 C CNN
F 3 "" H 3950 2750 50  0001 C CNN
	1    3950 2750
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5D1F62F9
P 4100 2850
F 0 "#PWR?" H 4100 2600 50  0001 C CNN
F 1 "GND" H 4100 2700 50  0000 C CNN
F 2 "" H 4100 2850 50  0001 C CNN
F 3 "" H 4100 2850 50  0001 C CNN
	1    4100 2850
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5D1F666E
P 6700 4300
F 0 "#PWR?" H 6700 4050 50  0001 C CNN
F 1 "GND" H 6700 4150 50  0000 C CNN
F 2 "" H 6700 4300 50  0001 C CNN
F 3 "" H 6700 4300 50  0001 C CNN
	1    6700 4300
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR?
U 1 1 5D1F668F
P 6800 4100
F 0 "#PWR?" H 6800 3850 50  0001 C CNN
F 1 "GND" H 6800 3950 50  0000 C CNN
F 2 "" H 6800 4100 50  0001 C CNN
F 3 "" H 6800 4100 50  0001 C CNN
	1    6800 4100
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR?
U 1 1 5D1F66A9
P 7000 4300
F 0 "#PWR?" H 7000 4050 50  0001 C CNN
F 1 "GND" H 7000 4150 50  0000 C CNN
F 2 "" H 7000 4300 50  0001 C CNN
F 3 "" H 7000 4300 50  0001 C CNN
	1    7000 4300
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR?
U 1 1 5D1F66C3
P 7100 4100
F 0 "#PWR?" H 7100 3850 50  0001 C CNN
F 1 "GND" H 7100 3950 50  0000 C CNN
F 2 "" H 7100 4100 50  0001 C CNN
F 3 "" H 7100 4100 50  0001 C CNN
	1    7100 4100
	-1   0    0    1   
$EndComp
Wire Wire Line
	7100 4300 7100 4100
Wire Wire Line
	6800 4100 6800 4300
Wire Wire Line
	3950 2750 4100 2750
$Comp
L +5V #PWR?
U 1 1 5D1F6D83
P 1900 1700
F 0 "#PWR?" H 1900 1550 50  0001 C CNN
F 1 "+5V" H 1900 1840 50  0000 C CNN
F 2 "" H 1900 1700 50  0001 C CNN
F 3 "" H 1900 1700 50  0001 C CNN
	1    1900 1700
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 5D1F6F77
P 4100 3050
F 0 "#PWR?" H 4100 2900 50  0001 C CNN
F 1 "+5V" H 4100 3190 50  0000 C CNN
F 2 "" H 4100 3050 50  0001 C CNN
F 3 "" H 4100 3050 50  0001 C CNN
	1    4100 3050
	0    -1   -1   0   
$EndComp
$Comp
L Screw_Terminal_1x02 J?
U 1 1 5D1F7228
P 8950 4900
F 0 "J?" H 8950 5150 50  0000 C TNN
F 1 "Screw_Terminal_1x02" V 8800 4900 50  0001 C TNN
F 2 "" H 8950 4675 50  0001 C CNN
F 3 "" H 8925 4900 50  0001 C CNN
	1    8950 4900
	-1   0    0    1   
$EndComp
$Comp
L D D?
U 1 1 5D1F72F6
P 8400 4850
F 0 "D?" H 8400 4950 50  0000 C CNN
F 1 "D" H 8400 4750 50  0000 C CNN
F 2 "" H 8400 4850 50  0001 C CNN
F 3 "" H 8400 4850 50  0001 C CNN
	1    8400 4850
	0    1    1    0   
$EndComp
$Comp
L D D?
U 1 1 5D1F733A
P 8400 5150
F 0 "D?" H 8400 5250 50  0000 C CNN
F 1 "D" H 8400 5050 50  0000 C CNN
F 2 "" H 8400 5150 50  0001 C CNN
F 3 "" H 8400 5150 50  0001 C CNN
	1    8400 5150
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5D1F75AC
P 8200 5300
F 0 "#PWR?" H 8200 5050 50  0001 C CNN
F 1 "GND" H 8200 5150 50  0000 C CNN
F 2 "" H 8200 5300 50  0001 C CNN
F 3 "" H 8200 5300 50  0001 C CNN
	1    8200 5300
	-1   0    0    1   
$EndComp
$Comp
L D D?
U 1 1 5D1F764D
P 8000 4650
F 0 "D?" H 8000 4750 50  0000 C CNN
F 1 "D" H 8000 4550 50  0000 C CNN
F 2 "" H 8000 4650 50  0001 C CNN
F 3 "" H 8000 4650 50  0001 C CNN
	1    8000 4650
	0    1    1    0   
$EndComp
$Comp
L D D?
U 1 1 5D1F7653
P 8000 4950
F 0 "D?" H 8000 5050 50  0000 C CNN
F 1 "D" H 8000 4850 50  0000 C CNN
F 2 "" H 8000 4950 50  0001 C CNN
F 3 "" H 8000 4950 50  0001 C CNN
	1    8000 4950
	0    1    1    0   
$EndComp
$Comp
L +BATT #PWR?
U 1 1 5D1F7659
P 8200 4500
F 0 "#PWR?" H 8200 4350 50  0001 C CNN
F 1 "+BATT" H 8200 4640 50  0000 C CNN
F 2 "" H 8200 4500 50  0001 C CNN
F 3 "" H 8200 4500 50  0001 C CNN
	1    8200 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 4500 8400 4500
Wire Wire Line
	8400 4500 8400 4700
Wire Wire Line
	8400 5300 8000 5300
Wire Wire Line
	8000 5300 8000 5100
Wire Wire Line
	7400 4800 8750 4800
Wire Wire Line
	7400 4800 7400 4900
Connection ~ 8000 4800
Wire Wire Line
	7400 5100 8200 5100
Wire Wire Line
	8200 5100 8200 5000
Wire Wire Line
	8200 5000 8750 5000
Connection ~ 8400 5000
$Comp
L Screw_Terminal_1x02 J?
U 1 1 5D1F96EE
P 8950 5900
F 0 "J?" H 8950 6150 50  0000 C TNN
F 1 "Screw_Terminal_1x02" V 8800 5900 50  0001 C TNN
F 2 "" H 8950 5675 50  0001 C CNN
F 3 "" H 8925 5900 50  0001 C CNN
	1    8950 5900
	-1   0    0    1   
$EndComp
$Comp
L D D?
U 1 1 5D1F96F4
P 8400 5850
F 0 "D?" H 8400 5950 50  0000 C CNN
F 1 "D" H 8400 5750 50  0000 C CNN
F 2 "" H 8400 5850 50  0001 C CNN
F 3 "" H 8400 5850 50  0001 C CNN
	1    8400 5850
	0    1    1    0   
$EndComp
$Comp
L D D?
U 1 1 5D1F96FA
P 8400 6150
F 0 "D?" H 8400 6250 50  0000 C CNN
F 1 "D" H 8400 6050 50  0000 C CNN
F 2 "" H 8400 6150 50  0001 C CNN
F 3 "" H 8400 6150 50  0001 C CNN
	1    8400 6150
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5D1F9700
P 8200 6300
F 0 "#PWR?" H 8200 6050 50  0001 C CNN
F 1 "GND" H 8200 6150 50  0000 C CNN
F 2 "" H 8200 6300 50  0001 C CNN
F 3 "" H 8200 6300 50  0001 C CNN
	1    8200 6300
	-1   0    0    1   
$EndComp
$Comp
L D D?
U 1 1 5D1F9706
P 8000 5650
F 0 "D?" H 8000 5750 50  0000 C CNN
F 1 "D" H 8000 5550 50  0000 C CNN
F 2 "" H 8000 5650 50  0001 C CNN
F 3 "" H 8000 5650 50  0001 C CNN
	1    8000 5650
	0    1    1    0   
$EndComp
$Comp
L D D?
U 1 1 5D1F970C
P 8000 5950
F 0 "D?" H 8000 6050 50  0000 C CNN
F 1 "D" H 8000 5850 50  0000 C CNN
F 2 "" H 8000 5950 50  0001 C CNN
F 3 "" H 8000 5950 50  0001 C CNN
	1    8000 5950
	0    1    1    0   
$EndComp
$Comp
L +BATT #PWR?
U 1 1 5D1F9712
P 8200 5500
F 0 "#PWR?" H 8200 5350 50  0001 C CNN
F 1 "+BATT" H 8200 5640 50  0000 C CNN
F 2 "" H 8200 5500 50  0001 C CNN
F 3 "" H 8200 5500 50  0001 C CNN
	1    8200 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 5500 8400 5500
Wire Wire Line
	8400 5500 8400 5700
Wire Wire Line
	8400 6300 8000 6300
Wire Wire Line
	8000 6300 8000 6100
Connection ~ 8000 5800
Connection ~ 8400 6000
Wire Wire Line
	7800 5800 8750 5800
Wire Wire Line
	7800 5800 7800 5500
Wire Wire Line
	7800 5500 7400 5500
Wire Wire Line
	7400 5700 7600 5700
Wire Wire Line
	7600 5700 7600 6000
Wire Wire Line
	7600 6000 8750 6000
$Comp
L MC34063AP U?
U 1 1 5D1F4C8B
P 2800 4800
F 0 "U?" H 2500 5150 50  0000 L CNN
F 1 "MC34063AP" H 2800 5150 50  0000 L CNN
F 2 "Housings_DIP:DIP-8_W7.62mm" H 2850 4350 50  0001 L CNN
F 3 "" H 3300 4700 50  0001 C CNN
	1    2800 4800
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 5D1F5B4E
P 6800 6100
F 0 "#PWR?" H 6800 5950 50  0001 C CNN
F 1 "+5V" H 6800 6240 50  0000 C CNN
F 2 "" H 6800 6100 50  0001 C CNN
F 3 "" H 6800 6100 50  0001 C CNN
	1    6800 6100
	-1   0    0    1   
$EndComp
$Comp
L +BATT #PWR?
U 1 1 5D1F6590
P 7000 6100
F 0 "#PWR?" H 7000 5950 50  0001 C CNN
F 1 "+BATT" H 7000 6240 50  0000 C CNN
F 2 "" H 7000 6100 50  0001 C CNN
F 3 "" H 7000 6100 50  0001 C CNN
	1    7000 6100
	-1   0    0    1   
$EndComp
$Comp
L +BATT #PWR?
U 1 1 5D1F72AB
P 1950 4600
F 0 "#PWR?" H 1950 4450 50  0001 C CNN
F 1 "+BATT" H 1950 4740 50  0000 C CNN
F 2 "" H 1950 4600 50  0001 C CNN
F 3 "" H 1950 4600 50  0001 C CNN
	1    1950 4600
	0    -1   -1   0   
$EndComp
$Comp
L CP C?
U 1 1 5D1F7612
P 2100 4750
F 0 "C?" H 2125 4850 50  0000 L CNN
F 1 "100uF" H 2125 4650 50  0000 L CNN
F 2 "" H 2138 4600 50  0001 C CNN
F 3 "" H 2100 4750 50  0001 C CNN
	1    2100 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 4600 1950 4600
$Comp
L R R?
U 1 1 5D1F7695
P 2800 4300
F 0 "R?" V 2880 4300 50  0000 C CNN
F 1 "R" V 2800 4300 50  0000 C CNN
F 2 "" V 2730 4300 50  0001 C CNN
F 3 "" H 2800 4300 50  0001 C CNN
	1    2800 4300
	0    1    1    0   
$EndComp
Wire Wire Line
	2950 4300 3200 4300
Wire Wire Line
	3200 4300 3200 4800
Connection ~ 3200 4600
Connection ~ 3200 4700
Wire Wire Line
	2650 4300 2400 4300
Wire Wire Line
	2400 4300 2400 4600
$Comp
L C C?
U 1 1 5D1F790F
P 2300 5150
F 0 "C?" H 2325 5250 50  0000 L CNN
F 1 "470pF" H 2325 5050 50  0000 L CNN
F 2 "" H 2338 5000 50  0001 C CNN
F 3 "" H 2300 5150 50  0001 C CNN
	1    2300 5150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5D1F7966
P 2100 4900
F 0 "#PWR?" H 2100 4650 50  0001 C CNN
F 1 "GND" H 2100 4750 50  0000 C CNN
F 2 "" H 2100 4900 50  0001 C CNN
F 3 "" H 2100 4900 50  0001 C CNN
	1    2100 4900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5D1F79B1
P 2300 5300
F 0 "#PWR?" H 2300 5050 50  0001 C CNN
F 1 "GND" H 2300 5150 50  0000 C CNN
F 2 "" H 2300 5300 50  0001 C CNN
F 3 "" H 2300 5300 50  0001 C CNN
	1    2300 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 5000 2400 5000
$Comp
L GND #PWR?
U 1 1 5D1F7A7A
P 2800 5300
F 0 "#PWR?" H 2800 5050 50  0001 C CNN
F 1 "GND" H 2800 5150 50  0000 C CNN
F 2 "" H 2800 5300 50  0001 C CNN
F 3 "" H 2800 5300 50  0001 C CNN
	1    2800 5300
	1    0    0    -1  
$EndComp
$Comp
L L L?
U 1 1 5D1F7AC0
P 3550 5000
F 0 "L?" V 3500 5000 50  0000 C CNN
F 1 "L" V 3625 5000 50  0000 C CNN
F 2 "" H 3550 5000 50  0001 C CNN
F 3 "" H 3550 5000 50  0001 C CNN
	1    3550 5000
	0    -1   -1   0   
$EndComp
$Comp
L D D?
U 1 1 5D1F7B83
P 3400 4850
F 0 "D?" H 3400 4950 50  0000 C CNN
F 1 "D" H 3400 4750 50  0000 C CNN
F 2 "" H 3400 4850 50  0001 C CNN
F 3 "" H 3400 4850 50  0001 C CNN
	1    3400 4850
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5D1F7BFB
P 3400 4700
F 0 "#PWR?" H 3400 4450 50  0001 C CNN
F 1 "GND" H 3400 4550 50  0000 C CNN
F 2 "" H 3400 4700 50  0001 C CNN
F 3 "" H 3400 4700 50  0001 C CNN
	1    3400 4700
	-1   0    0    1   
$EndComp
Wire Wire Line
	3400 5000 3200 5000
$Comp
L R R?
U 1 1 5D1F7C74
P 3800 5450
F 0 "R?" V 3880 5450 50  0000 C CNN
F 1 "R" V 3800 5450 50  0000 C CNN
F 2 "" V 3730 5450 50  0001 C CNN
F 3 "" H 3800 5450 50  0001 C CNN
	1    3800 5450
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5D1F7D89
P 3800 5150
F 0 "R?" V 3880 5150 50  0000 C CNN
F 1 "R" V 3800 5150 50  0000 C CNN
F 2 "" V 3730 5150 50  0001 C CNN
F 3 "" H 3800 5150 50  0001 C CNN
	1    3800 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 5000 4400 5000
Wire Wire Line
	3800 5300 3200 5300
Wire Wire Line
	3200 5300 3200 5100
$Comp
L GND #PWR?
U 1 1 5D1F7EAD
P 3800 5600
F 0 "#PWR?" H 3800 5350 50  0001 C CNN
F 1 "GND" H 3800 5450 50  0000 C CNN
F 2 "" H 3800 5600 50  0001 C CNN
F 3 "" H 3800 5600 50  0001 C CNN
	1    3800 5600
	1    0    0    -1  
$EndComp
$Comp
L CP C?
U 1 1 5D1F7EE8
P 4150 5150
F 0 "C?" H 4175 5250 50  0000 L CNN
F 1 "CP" H 4175 5050 50  0000 L CNN
F 2 "" H 4188 5000 50  0001 C CNN
F 3 "" H 4150 5150 50  0001 C CNN
	1    4150 5150
	1    0    0    -1  
$EndComp
Connection ~ 3800 5000
Connection ~ 4150 5000
$Comp
L GND #PWR?
U 1 1 5D1F7F4D
P 4150 5300
F 0 "#PWR?" H 4150 5050 50  0001 C CNN
F 1 "GND" H 4150 5150 50  0000 C CNN
F 2 "" H 4150 5300 50  0001 C CNN
F 3 "" H 4150 5300 50  0001 C CNN
	1    4150 5300
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 5D1F8487
P 4400 5000
F 0 "#PWR?" H 4400 4850 50  0001 C CNN
F 1 "+5V" H 4400 5140 50  0000 C CNN
F 2 "" H 4400 5000 50  0001 C CNN
F 3 "" H 4400 5000 50  0001 C CNN
	1    4400 5000
	0    1    1    0   
$EndComp
Text GLabel 6900 1350 2    60   Input ~ 0
IntMPU
$Comp
L CONN_01X08 J?
U 1 1 5D1FB414
P 9550 1600
F 0 "J?" H 9550 2050 50  0000 C CNN
F 1 "CONN_01X08" V 9650 1600 50  0000 C CNN
F 2 "" H 9550 1600 50  0001 C CNN
F 3 "" H 9550 1600 50  0001 C CNN
	1    9550 1600
	1    0    0    -1  
$EndComp
Text Notes 9750 1600 0    60   ~ 0
OrangePi\n
$Comp
L CONN_01X03 J?
U 1 1 5D1FC1C7
P 9550 2400
F 0 "J?" H 9550 2600 50  0000 C CNN
F 1 "CONN_01X03" V 9650 2400 50  0000 C CNN
F 2 "" H 9550 2400 50  0001 C CNN
F 3 "" H 9550 2400 50  0001 C CNN
	1    9550 2400
	1    0    0    -1  
$EndComp
Text Notes 9800 2450 0    60   ~ 0
Servocomando\n
$Comp
L I2c_LCD U?
U 1 1 5D1FCAAB
P 1100 3050
F 0 "U?" H 1875 3350 60  0000 C CNN
F 1 "I2c_LCD" H 1425 3150 60  0000 C CNN
F 2 "" H 1050 3050 60  0001 C CNN
F 3 "" H 1050 3050 60  0001 C CNN
	1    1100 3050
	0    1    1    0   
$EndComp
Wire Wire Line
	1650 3200 2350 3200
Wire Wire Line
	2350 3200 2350 1950
Connection ~ 2350 1950
Wire Wire Line
	2500 2050 2500 3350
Wire Wire Line
	2500 3350 1650 3350
Connection ~ 2500 2050
$Comp
L +5V #PWR?
U 1 1 5D1FCF78
P 1650 3500
F 0 "#PWR?" H 1650 3350 50  0001 C CNN
F 1 "+5V" H 1650 3640 50  0000 C CNN
F 2 "" H 1650 3500 50  0001 C CNN
F 3 "" H 1650 3500 50  0001 C CNN
	1    1650 3500
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5D1FD046
P 1650 3650
F 0 "#PWR?" H 1650 3400 50  0001 C CNN
F 1 "GND" H 1650 3500 50  0000 C CNN
F 2 "" H 1650 3650 50  0001 C CNN
F 3 "" H 1650 3650 50  0001 C CNN
	1    1650 3650
	0    -1   -1   0   
$EndComp
Text GLabel 6900 1550 2    60   Input ~ 0
WDFOp
Wire Wire Line
	6700 1350 6900 1350
Wire Wire Line
	7250 1450 6700 1450
Text GLabel 7200 2450 2    60   Input ~ 0
WDTOp
$Comp
L +5V #PWR?
U 1 1 5D1FF9C4
P 9350 1250
F 0 "#PWR?" H 9350 1100 50  0001 C CNN
F 1 "+5V" H 9350 1390 50  0000 C CNN
F 2 "" H 9350 1250 50  0001 C CNN
F 3 "" H 9350 1250 50  0001 C CNN
	1    9350 1250
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5D1FFDD3
P 9150 1350
F 0 "#PWR?" H 9150 1100 50  0001 C CNN
F 1 "GND" H 9150 1200 50  0000 C CNN
F 2 "" H 9150 1350 50  0001 C CNN
F 3 "" H 9150 1350 50  0001 C CNN
	1    9150 1350
	0    1    1    0   
$EndComp
Text GLabel 8850 1450 0    60   Input ~ 0
WDFOp
Text GLabel 9300 1550 0    60   Input ~ 0
WDTOp
Wire Wire Line
	9150 1350 9350 1350
Wire Wire Line
	9350 1450 8850 1450
Wire Wire Line
	9300 1550 9350 1550
Text GLabel 7200 2050 2    60   Input ~ 0
Servo
Text GLabel 9350 2500 0    60   Input ~ 0
Servo
$Comp
L GND #PWR?
U 1 1 5D2011B1
P 9350 2300
F 0 "#PWR?" H 9350 2050 50  0001 C CNN
F 1 "GND" H 9350 2150 50  0000 C CNN
F 2 "" H 9350 2300 50  0001 C CNN
F 3 "" H 9350 2300 50  0001 C CNN
	1    9350 2300
	0    1    1    0   
$EndComp
$Comp
L +5V #PWR?
U 1 1 5D20120D
P 9050 2400
F 0 "#PWR?" H 9050 2250 50  0001 C CNN
F 1 "+5V" H 9050 2540 50  0000 C CNN
F 2 "" H 9050 2400 50  0001 C CNN
F 3 "" H 9050 2400 50  0001 C CNN
	1    9050 2400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9350 2400 9050 2400
Text GLabel 4100 1350 0    60   Input ~ 0
Rx
Text GLabel 3900 1450 0    60   Input ~ 0
Tx
Wire Wire Line
	3900 1450 4100 1450
Text GLabel 8900 1650 0    60   Input ~ 0
Rx
Text GLabel 9300 1750 0    60   Input ~ 0
Tx
Wire Wire Line
	9350 1750 9300 1750
Wire Wire Line
	9350 1650 8900 1650
$Comp
L R 20K
U 1 1 5D234EDB
P 3100 2500
F 0 "20K" V 3180 2500 50  0000 C CNN
F 1 "R" V 3100 2500 50  0000 C CNN
F 2 "" V 3030 2500 50  0001 C CNN
F 3 "" H 3100 2500 50  0001 C CNN
	1    3100 2500
	1    0    0    -1  
$EndComp
$Comp
L R 10K
U 1 1 5D234F28
P 3100 2800
F 0 "10K" V 3180 2800 50  0000 C CNN
F 1 "R" V 3100 2800 50  0000 C CNN
F 2 "" V 3030 2800 50  0001 C CNN
F 3 "" H 3100 2800 50  0001 C CNN
	1    3100 2800
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR?
U 1 1 5D235962
P 3100 2350
F 0 "#PWR?" H 3100 2200 50  0001 C CNN
F 1 "+BATT" H 3100 2490 50  0000 C CNN
F 2 "" H 3100 2350 50  0001 C CNN
F 3 "" H 3100 2350 50  0001 C CNN
	1    3100 2350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5D2359C4
P 3100 2950
F 0 "#PWR?" H 3100 2700 50  0001 C CNN
F 1 "GND" H 3100 2800 50  0000 C CNN
F 2 "" H 3100 2950 50  0001 C CNN
F 3 "" H 3100 2950 50  0001 C CNN
	1    3100 2950
	1    0    0    -1  
$EndComp
Text Notes 3000 3400 1    60   ~ 0
Controllo Carica Batteria
Wire Wire Line
	3100 2650 3650 2650
Wire Wire Line
	3650 2650 3650 2450
Wire Wire Line
	3650 2450 4100 2450
Wire Wire Line
	6900 1950 6700 1950
Wire Wire Line
	7200 2050 6700 2050
Wire Wire Line
	7200 2250 6700 2250
Text Notes 9150 4950 0    60   ~ 0
Motor R
Text Notes 9150 5950 0    60   ~ 0
Motor L\n
Wire Wire Line
	7200 2450 6700 2450
$EndSCHEMATC
