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
LIBS:switches
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
L AT24CS02-SSHM IC1
U 1 1 589C1FFD
P 5600 3250
F 0 "IC1" H 5400 3500 50  0000 C CNN
F 1 "AT24CS02-SSHM" H 5900 2950 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 5600 3250 50  0001 C CIN
F 3 "" H 5600 3250 50  0000 C CNN
	1    5600 3250
	-1   0    0    -1  
$EndComp
$Comp
L +3.3V #PWR045
U 1 1 589C205E
P 5600 2850
F 0 "#PWR045" H 5600 2700 50  0001 C CNN
F 1 "+3.3V" H 5600 2990 50  0000 C CNN
F 2 "" H 5600 2850 50  0000 C CNN
F 3 "" H 5600 2850 50  0000 C CNN
	1    5600 2850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR046
U 1 1 589C2085
P 5600 3700
F 0 "#PWR046" H 5600 3450 50  0001 C CNN
F 1 "GND" H 5600 3550 50  0000 C CNN
F 2 "" H 5600 3700 50  0000 C CNN
F 3 "" H 5600 3700 50  0000 C CNN
	1    5600 3700
	1    0    0    -1  
$EndComp
Text HLabel 5100 3300 0    60   Input ~ 0
scl
Wire Wire Line
	5100 3300 5200 3300
Text HLabel 5100 3150 0    60   BiDi ~ 0
sda
Wire Wire Line
	5100 3150 5200 3150
Wire Wire Line
	6050 3300 6000 3300
$Comp
L GND #PWR047
U 1 1 589C239F
P 6050 3700
F 0 "#PWR047" H 6050 3450 50  0001 C CNN
F 1 "GND" H 6050 3550 50  0000 C CNN
F 2 "" H 6050 3700 50  0000 C CNN
F 3 "" H 6050 3700 50  0000 C CNN
	1    6050 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 3100 6050 3700
Wire Wire Line
	6000 3200 6050 3200
Connection ~ 6050 3300
Wire Wire Line
	6000 3100 6050 3100
Connection ~ 6050 3200
Wire Wire Line
	6000 3450 6050 3450
Connection ~ 6050 3450
Wire Wire Line
	5600 3700 5600 3650
Wire Wire Line
	5600 2900 5600 2850
$EndSCHEMATC
