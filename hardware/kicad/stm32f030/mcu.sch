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
LIBS:button
LIBS:con-wago
LIBS:gdt
LIBS:irlz44nto220v
LIBS:jumpers
LIBS:relay
LIBS:transformer
LIBS:stm32
LIBS:stm32f030-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 5
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L STM32F030F4Px U2
U 1 1 5884C959
P 6050 3750
F 0 "U2" H 4450 4675 50  0000 L BNN
F 1 "STM32F030F4Px" H 7650 4675 50  0000 R BNN
F 2 "Housings_SSOP:TSSOP-20_4.4x6.5mm_Pitch0.65mm" H 7650 4625 50  0001 R TNN
F 3 "" H 6050 3750 50  0000 C CNN
	1    6050 3750
	-1   0    0    -1  
$EndComp
$Comp
L +3.3V #PWR043
U 1 1 5884CABB
P 6050 2600
F 0 "#PWR043" H 6050 2450 50  0001 C CNN
F 1 "+3.3V" H 6050 2740 50  0000 C CNN
F 2 "" H 6050 2600 50  0000 C CNN
F 3 "" H 6050 2600 50  0000 C CNN
	1    6050 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 2600 6050 2750
Wire Wire Line
	6150 2750 6150 2650
Wire Wire Line
	6150 2650 6050 2650
Connection ~ 6050 2650
$Comp
L GND #PWR044
U 1 1 5884CAD9
P 6050 4900
F 0 "#PWR044" H 6050 4650 50  0001 C CNN
F 1 "GND" H 6050 4750 50  0000 C CNN
F 2 "" H 6050 4900 50  0000 C CNN
F 3 "" H 6050 4900 50  0000 C CNN
	1    6050 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 4650 6050 4900
$Comp
L GND #PWR045
U 1 1 5884CAF5
P 9350 4900
F 0 "#PWR045" H 9350 4650 50  0001 C CNN
F 1 "GND" H 9350 4750 50  0000 C CNN
F 2 "" H 9350 4900 50  0000 C CNN
F 3 "" H 9350 4900 50  0000 C CNN
	1    9350 4900
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR046
U 1 1 5884CB05
P 9350 4600
F 0 "#PWR046" H 9350 4450 50  0001 C CNN
F 1 "+3.3V" H 9350 4740 50  0000 C CNN
F 2 "" H 9350 4600 50  0000 C CNN
F 3 "" H 9350 4600 50  0000 C CNN
	1    9350 4600
	1    0    0    -1  
$EndComp
$Comp
L C_Small C16
U 1 1 5884CB14
P 9350 4750
F 0 "C16" H 9360 4820 50  0000 L CNN
F 1 "100n" H 9360 4670 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9350 4750 50  0001 C CNN
F 3 "" H 9350 4750 50  0000 C CNN
	1    9350 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9350 4600 9350 4650
Wire Wire Line
	9350 4850 9350 4900
Text HLabel 3050 3650 0    60   Input ~ 0
active1
Text HLabel 8450 4250 2    60   Input ~ 0
active2
Wire Wire Line
	7750 4250 8450 4250
Text HLabel 3050 3750 0    60   Input ~ 0
osc1
Text HLabel 3050 3850 0    60   Input ~ 0
osc2
$Comp
L Crystal_Small Y1
U 1 1 5884DD83
P 8000 4450
F 0 "Y1" H 8000 4550 50  0000 C CNN
F 1 "8MHz" H 8000 4350 50  0000 C CNN
F 2 "Crystals:Crystal_HC49-SD_SMD" H 8000 4450 50  0001 C CNN
F 3 "" H 8000 4450 50  0000 C CNN
	1    8000 4450
	1    0    0    -1  
$EndComp
$Comp
L C_Small C15
U 1 1 5884DE3A
P 8150 4700
F 0 "C15" H 8160 4770 50  0000 L CNN
F 1 "22p" H 8160 4620 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 8150 4700 50  0001 C CNN
F 3 "" H 8150 4700 50  0000 C CNN
	1    8150 4700
	1    0    0    -1  
$EndComp
$Comp
L C_Small C14
U 1 1 5884DE87
P 7850 4700
F 0 "C14" H 7860 4770 50  0000 L CNN
F 1 "22p" H 7860 4620 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 7850 4700 50  0001 C CNN
F 3 "" H 7850 4700 50  0000 C CNN
	1    7850 4700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR047
U 1 1 5884DEF5
P 7850 4900
F 0 "#PWR047" H 7850 4650 50  0001 C CNN
F 1 "GND" H 7850 4750 50  0000 C CNN
F 2 "" H 7850 4900 50  0000 C CNN
F 3 "" H 7850 4900 50  0000 C CNN
	1    7850 4900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR048
U 1 1 5884DF18
P 8150 4900
F 0 "#PWR048" H 8150 4650 50  0001 C CNN
F 1 "GND" H 8150 4750 50  0000 C CNN
F 2 "" H 8150 4900 50  0000 C CNN
F 3 "" H 8150 4900 50  0000 C CNN
	1    8150 4900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR049
U 1 1 5884DF65
P 7850 3650
F 0 "#PWR049" H 7850 3400 50  0001 C CNN
F 1 "GND" H 7850 3500 50  0000 C CNN
F 2 "" H 7850 3650 50  0000 C CNN
F 3 "" H 7850 3650 50  0000 C CNN
	1    7850 3650
	1    0    0    -1  
$EndComp
$Comp
L TACTILE_SW SW3
U 1 1 5884DF9D
P 8450 3150
F 0 "SW3" H 8600 3260 50  0000 C CNN
F 1 "RESET" H 8470 3010 50  0000 C CNN
F 2 "Buttons_Switches_THT:SW_PUSH_6mm" H 8450 3150 50  0001 C CNN
F 3 "" H 8450 3150 50  0000 C CNN
	1    8450 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 3150 8250 3150
$Comp
L GND #PWR050
U 1 1 5884E164
P 8800 3450
F 0 "#PWR050" H 8800 3200 50  0001 C CNN
F 1 "GND" H 8800 3300 50  0000 C CNN
F 2 "" H 8800 3450 50  0000 C CNN
F 3 "" H 8800 3450 50  0000 C CNN
	1    8800 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 3150 8800 3150
Wire Wire Line
	8800 3150 8800 3450
$Comp
L R_Small R25
U 1 1 5884E1A2
P 8100 2850
F 0 "R25" H 8130 2870 50  0000 L CNN
F 1 "10k" H 8130 2810 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 8100 2850 50  0001 C CNN
F 3 "" H 8100 2850 50  0000 C CNN
	1    8100 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 2950 8100 3800
Connection ~ 8100 3150
$Comp
L +3.3V #PWR051
U 1 1 5884E1FE
P 8100 2600
F 0 "#PWR051" H 8100 2450 50  0001 C CNN
F 1 "+3.3V" H 8100 2740 50  0000 C CNN
F 2 "" H 8100 2600 50  0000 C CNN
F 3 "" H 8100 2600 50  0000 C CNN
	1    8100 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 2600 8100 2750
$Comp
L CONN_01X03 P9
U 1 1 5884E468
P 3850 4050
F 0 "P9" H 4000 4200 50  0000 C CNN
F 1 "I2C" V 3950 3950 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 3850 4050 50  0001 C CNN
F 3 "" H 3850 4050 50  0000 C CNN
	1    3850 4050
	-1   0    0    -1  
$EndComp
$Comp
L CONN_01X03 P10
U 1 1 5884E5D6
P 3850 4500
F 0 "P10" H 3850 4300 50  0000 C CNN
F 1 "SWD" V 3950 4550 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 3850 4500 50  0001 C CNN
F 3 "" H 3850 4500 50  0000 C CNN
	1    3850 4500
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4050 4400 4250 4400
Wire Wire Line
	4250 4400 4250 4150
Wire Wire Line
	4050 4500 4300 4500
Wire Wire Line
	4050 4150 4100 4150
Wire Wire Line
	4100 3550 4100 4900
Wire Wire Line
	4100 4600 4050 4600
$Comp
L GND #PWR052
U 1 1 5884E75E
P 4100 4900
F 0 "#PWR052" H 4100 4650 50  0001 C CNN
F 1 "GND" H 4100 4750 50  0000 C CNN
F 2 "" H 4100 4900 50  0000 C CNN
F 3 "" H 4100 4900 50  0000 C CNN
	1    4100 4900
	1    0    0    -1  
$EndComp
Connection ~ 4100 4600
$Comp
L CONN_01X03 P8
U 1 1 5884E804
P 3850 3450
F 0 "P8" H 3850 3650 50  0000 C CNN
F 1 "UART" V 3950 3450 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 3850 3450 50  0001 C CNN
F 3 "" H 3850 3450 50  0000 C CNN
	1    3850 3450
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4050 3550 4100 3550
Connection ~ 4100 4150
Wire Wire Line
	7850 4050 7850 4600
Wire Wire Line
	7850 4050 7750 4050
Wire Wire Line
	8150 3950 8150 4600
Wire Wire Line
	8150 3950 7750 3950
Wire Wire Line
	7900 4450 7850 4450
Connection ~ 7850 4450
Wire Wire Line
	8100 4450 8150 4450
Connection ~ 8150 4450
Wire Wire Line
	8150 4900 8150 4800
Wire Wire Line
	7850 4900 7850 4800
Wire Wire Line
	4250 4150 4350 4150
Wire Wire Line
	4300 4500 4300 4250
Wire Wire Line
	4300 4250 4350 4250
Wire Wire Line
	4050 4050 4350 4050
Wire Wire Line
	4050 3950 4350 3950
Wire Wire Line
	3050 3850 4350 3850
Wire Wire Line
	3050 3750 4350 3750
Wire Wire Line
	3050 3650 4350 3650
Wire Wire Line
	4050 3450 4350 3450
Wire Wire Line
	4050 3350 4350 3350
Text HLabel 3050 3100 0    60   Output ~ 0
scl
Text HLabel 3050 3000 0    60   BiDi ~ 0
sda
Text HLabel 8450 3800 2    60   Output ~ 0
reset
Wire Wire Line
	8100 3800 8450 3800
Wire Wire Line
	3050 3100 4150 3100
Wire Wire Line
	4150 3100 4150 3950
Connection ~ 4150 3950
Wire Wire Line
	3050 3000 4250 3000
Wire Wire Line
	4250 2850 4250 4050
Connection ~ 4250 4050
$Comp
L R_Small R26
U 1 1 5885548F
P 4000 2750
F 0 "R26" H 4030 2770 50  0000 L CNN
F 1 "4.7k" H 4030 2710 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 4000 2750 50  0001 C CNN
F 3 "" H 4000 2750 50  0000 C CNN
	1    4000 2750
	1    0    0    -1  
$EndComp
$Comp
L R_Small R27
U 1 1 58855534
P 4250 2750
F 0 "R27" H 4280 2770 50  0000 L CNN
F 1 "4.7k" H 4280 2710 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 4250 2750 50  0001 C CNN
F 3 "" H 4250 2750 50  0000 C CNN
	1    4250 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 2850 4000 3100
Connection ~ 4000 3100
Connection ~ 4250 3000
$Comp
L +3.3V #PWR053
U 1 1 58855640
P 4000 2600
F 0 "#PWR053" H 4000 2450 50  0001 C CNN
F 1 "+3.3V" H 4000 2740 50  0000 C CNN
F 2 "" H 4000 2600 50  0000 C CNN
F 3 "" H 4000 2600 50  0000 C CNN
	1    4000 2600
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR054
U 1 1 5885566F
P 4250 2600
F 0 "#PWR054" H 4250 2450 50  0001 C CNN
F 1 "+3.3V" H 4250 2740 50  0000 C CNN
F 2 "" H 4250 2600 50  0000 C CNN
F 3 "" H 4250 2600 50  0000 C CNN
	1    4250 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 2600 4000 2650
Wire Wire Line
	4250 2600 4250 2650
$Comp
L JP3x1 JP3
U 1 1 588558E2
P 7950 3250
F 0 "JP3" H 7950 2950 60  0000 C CNN
F 1 "JP3x1" H 8000 2950 60  0001 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 7950 3250 60  0001 C CNN
F 3 "" H 7950 3250 60  0000 C CNN
	1    7950 3250
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR055
U 1 1 5885595B
P 7850 3050
F 0 "#PWR055" H 7850 2900 50  0001 C CNN
F 1 "+3.3V" H 7850 3190 50  0000 C CNN
F 2 "" H 7850 3050 50  0000 C CNN
F 3 "" H 7850 3050 50  0000 C CNN
	1    7850 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 3050 7850 3250
Wire Wire Line
	7850 3350 7750 3350
Wire Wire Line
	7850 3450 7850 3650
$EndSCHEMATC
