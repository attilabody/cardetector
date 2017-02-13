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
Sheet 2 6
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
L TRANSF_2X2 T2
U 1 1 588475B9
P 2600 5650
F 0 "T2" H 2600 5900 50  0000 C CNN
F 1 "78601/9C" H 2600 5350 50  0000 C CNN
F 2 "cardetector:Murata_Impulse_Transformer" H 2600 5650 50  0001 C CNN
F 3 "" H 2600 5650 50  0000 C CNN
	1    2600 5650
	-1   0    0    1   
$EndComp
$Comp
L R_Small R14
U 1 1 588475BA
P 1850 5250
F 0 "R14" V 1750 5100 50  0000 L CNN
F 1 "1.8M" V 1750 5250 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 1850 5250 50  0001 C CNN
F 3 "" H 1850 5250 50  0000 C CNN
	1    1850 5250
	1    0    0    1   
$EndComp
$Comp
L GDT GDT3
U 1 1 588475BC
P 2100 5650
F 0 "GDT3" H 2110 5540 60  0000 C CNN
F 1 "GDT" H 2100 5750 60  0000 C CNN
F 2 "cardetector:GDT" H 2225 5825 60  0001 C CNN
F 3 "" H 2225 5825 60  0000 C CNN
	1    2100 5650
	0    1    -1   0   
$EndComp
$Comp
L GDT GDT4
U 1 1 588475BD
P 2100 5250
F 0 "GDT4" H 2110 5140 60  0000 C CNN
F 1 "GDT" H 2100 5350 60  0000 C CNN
F 2 "cardetector:GDT" H 2225 5425 60  0001 C CNN
F 3 "" H 2225 5425 60  0000 C CNN
	1    2100 5250
	0    1    -1   0   
$EndComp
$Comp
L CONN_01X03 P3
U 1 1 588475BE
P 1350 5650
F 0 "P3" H 1350 5850 50  0000 C CNN
F 1 "LOOP2" V 1450 5650 50  0000 C CNN
F 2 "Connectors:bornier3" H 1350 5650 50  0001 C CNN
F 3 "" H 1350 5650 50  0000 C CNN
	1    1350 5650
	-1   0    0    -1  
$EndComp
$Comp
L C_Small C10
U 1 1 588475BF
P 3000 6450
F 0 "C10" H 3010 6520 50  0000 L CNN
F 1 "100n" H 3010 6370 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3000 6450 50  0001 C CNN
F 3 "" H 3000 6450 50  0000 C CNN
	1    3000 6450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 588475C0
P 3000 6650
F 0 "#PWR08" H 3000 6400 50  0001 C CNN
F 1 "GND" H 3000 6500 50  0000 C CNN
F 2 "" H 3000 6650 50  0000 C CNN
F 3 "" H 3000 6650 50  0000 C CNN
	1    3000 6650
	1    0    0    -1  
$EndComp
$Comp
L C_Small C11
U 1 1 588475C1
P 3200 6450
F 0 "C11" H 3210 6520 50  0000 L CNN
F 1 "47n" H 3210 6370 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3200 6450 50  0001 C CNN
F 3 "" H 3200 6450 50  0000 C CNN
	1    3200 6450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 588475C2
P 3200 6650
F 0 "#PWR09" H 3200 6400 50  0001 C CNN
F 1 "GND" H 3200 6500 50  0000 C CNN
F 2 "" H 3200 6650 50  0000 C CNN
F 3 "" H 3200 6650 50  0000 C CNN
	1    3200 6650
	1    0    0    -1  
$EndComp
$Comp
L C_Small C12
U 1 1 588475C3
P 3400 6450
F 0 "C12" H 3410 6520 50  0000 L CNN
F 1 "33n" H 3410 6370 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3400 6450 50  0001 C CNN
F 3 "" H 3400 6450 50  0000 C CNN
	1    3400 6450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR010
U 1 1 588475C4
P 3400 6650
F 0 "#PWR010" H 3400 6400 50  0001 C CNN
F 1 "GND" H 3400 6500 50  0000 C CNN
F 2 "" H 3400 6650 50  0000 C CNN
F 3 "" H 3400 6650 50  0000 C CNN
	1    3400 6650
	1    0    0    -1  
$EndComp
$Comp
L C_Small C13
U 1 1 588475C5
P 3600 6450
F 0 "C13" H 3610 6520 50  0000 L CNN
F 1 "22n" H 3610 6370 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3600 6450 50  0001 C CNN
F 3 "" H 3600 6450 50  0000 C CNN
	1    3600 6450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR011
U 1 1 588475C6
P 3600 6650
F 0 "#PWR011" H 3600 6400 50  0001 C CNN
F 1 "GND" H 3600 6500 50  0000 C CNN
F 2 "" H 3600 6650 50  0000 C CNN
F 3 "" H 3600 6650 50  0000 C CNN
	1    3600 6650
	1    0    0    -1  
$EndComp
$Comp
L R_Small R13
U 1 1 588475C9
P 3650 5850
F 0 "R13" V 3550 5700 50  0000 L CNN
F 1 "1k" V 3550 5900 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 3650 5850 50  0001 C CNN
F 3 "" H 3650 5850 50  0000 C CNN
	1    3650 5850
	0    1    1    0   
$EndComp
$Comp
L BSS138 Q3
U 1 1 588475CA
P 4000 5800
F 0 "Q3" H 3850 5950 50  0000 L CNN
F 1 "BSS138" H 3800 5550 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 4200 5725 50  0001 L CIN
F 3 "" H 4000 5800 50  0000 L CNN
	1    4000 5800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR012
U 1 1 588475CB
P 4100 6650
F 0 "#PWR012" H 4100 6400 50  0001 C CNN
F 1 "GND" H 4100 6500 50  0000 C CNN
F 2 "" H 4100 6650 50  0000 C CNN
F 3 "" H 4100 6650 50  0000 C CNN
	1    4100 6650
	1    0    0    -1  
$EndComp
$Comp
L R_Small R9
U 1 1 588475CC
P 4100 4850
F 0 "R9" V 4000 4700 50  0000 L CNN
F 1 "150" V 4000 4850 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 4100 4850 50  0001 C CNN
F 3 "" H 4100 4850 50  0000 C CNN
	1    4100 4850
	1    0    0    -1  
$EndComp
$Comp
L R_Small R8
U 1 1 588475CD
P 3900 4850
F 0 "R8" V 3800 4700 50  0000 L CNN
F 1 "150" V 3800 4850 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 3900 4850 50  0001 C CNN
F 3 "" H 3900 4850 50  0000 C CNN
	1    3900 4850
	1    0    0    -1  
$EndComp
$Comp
L BC817-40 Q4
U 1 1 588475CE
P 4900 6050
F 0 "Q4" H 4750 6200 50  0000 L CNN
F 1 "BC817-40" H 4550 5800 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 5100 5975 50  0001 L CIN
F 3 "" H 4900 6050 50  0000 L CNN
	1    4900 6050
	1    0    0    -1  
$EndComp
$Comp
L R_Small R12
U 1 1 588475CF
P 4700 5700
F 0 "R12" V 4600 5500 50  0000 L CNN
F 1 "4.7k" V 4600 5700 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 4700 5700 50  0001 C CNN
F 3 "" H 4700 5700 50  0000 C CNN
	1    4700 5700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 588475D0
P 5000 6650
F 0 "#PWR013" H 5000 6400 50  0001 C CNN
F 1 "GND" H 5000 6500 50  0000 C CNN
F 2 "" H 5000 6650 50  0000 C CNN
F 3 "" H 5000 6650 50  0000 C CNN
	1    5000 6650
	1    0    0    -1  
$EndComp
$Comp
L R_Small R10
U 1 1 588475D1
P 5000 4850
F 0 "R10" V 4900 4700 50  0000 L CNN
F 1 "1k" V 4900 4900 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 5000 4850 50  0001 C CNN
F 3 "" H 5000 4850 50  0000 C CNN
	1    5000 4850
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR014
U 1 1 588475D2
P 4100 4450
F 0 "#PWR014" H 4100 4300 50  0001 C CNN
F 1 "+5V" H 4100 4590 50  0000 C CNN
F 2 "" H 4100 4450 50  0000 C CNN
F 3 "" H 4100 4450 50  0000 C CNN
	1    4100 4450
	1    0    0    -1  
$EndComp
$Comp
L C_Small C9
U 1 1 588475D3
P 4350 6150
F 0 "C9" H 4360 6220 50  0000 L CNN
F 1 "1u" H 4360 6070 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 4350 6150 50  0001 C CNN
F 3 "" H 4350 6150 50  0000 C CNN
	1    4350 6150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR015
U 1 1 588475D4
P 4350 6650
F 0 "#PWR015" H 4350 6400 50  0001 C CNN
F 1 "GND" H 4350 6500 50  0000 C CNN
F 2 "" H 4350 6650 50  0000 C CNN
F 3 "" H 4350 6650 50  0000 C CNN
	1    4350 6650
	1    0    0    -1  
$EndComp
$Comp
L R_Small R11
U 1 1 588475D8
P 5650 5550
F 0 "R11" V 5750 5550 50  0000 L CNN
F 1 "4.7k" V 5750 5350 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 5650 5550 50  0001 C CNN
F 3 "" H 5650 5550 50  0000 C CNN
	1    5650 5550
	0    1    1    0   
$EndComp
Text HLabel 5950 5550 2    60   Input ~ 0
osc1
$Comp
L D_Zener_Small ZD3
U 1 1 58847C58
P 3200 5350
F 0 "ZD3" H 3200 5440 50  0000 C CNN
F 1 "24V" H 3200 5260 50  0000 C CNN
F 2 "Diodes_SMD:D_MiniMELF_Standard" V 3200 5350 50  0001 C CNN
F 3 "" V 3200 5350 50  0000 C CNN
	1    3200 5350
	0    -1   -1   0   
$EndComp
$Comp
L D_Zener_Small ZD4
U 1 1 58847EDD
P 3200 5650
F 0 "ZD4" H 3200 5740 50  0000 C CNN
F 1 "24V" H 3200 5560 50  0000 C CNN
F 2 "Diodes_SMD:D_MiniMELF_Standard" V 3200 5650 50  0001 C CNN
F 3 "" V 3200 5650 50  0000 C CNN
	1    3200 5650
	0    1    1    0   
$EndComp
$Comp
L TRANSF_2X2 T1
U 1 1 5884A6CC
P 2600 2550
F 0 "T1" H 2600 2800 50  0000 C CNN
F 1 "78601/9C" H 2600 2250 50  0000 C CNN
F 2 "cardetector:Murata_Impulse_Transformer" H 2600 2550 50  0001 C CNN
F 3 "" H 2600 2550 50  0000 C CNN
	1    2600 2550
	-1   0    0    -1  
$EndComp
$Comp
L R_Small R7
U 1 1 5884A6D3
P 1850 2150
F 0 "R7" V 1750 2000 50  0000 L CNN
F 1 "1.8M" V 1750 2150 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 1850 2150 50  0001 C CNN
F 3 "" H 1850 2150 50  0000 C CNN
	1    1850 2150
	1    0    0    1   
$EndComp
$Comp
L GDT GDT1
U 1 1 5884A6E0
P 2100 2150
F 0 "GDT1" H 2110 2040 60  0000 C CNN
F 1 "GDT" H 2100 2250 60  0000 C CNN
F 2 "cardetector:GDT" H 2225 2325 60  0001 C CNN
F 3 "" H 2225 2325 60  0000 C CNN
	1    2100 2150
	0    1    -1   0   
$EndComp
$Comp
L GDT GDT2
U 1 1 5884A6E7
P 2100 2550
F 0 "GDT2" H 2110 2440 60  0000 C CNN
F 1 "GDT" H 2100 2650 60  0000 C CNN
F 2 "cardetector:GDT" H 2225 2725 60  0001 C CNN
F 3 "" H 2225 2725 60  0000 C CNN
	1    2100 2550
	0    1    -1   0   
$EndComp
$Comp
L CONN_01X03 P2
U 1 1 5884A6EE
P 1350 2550
F 0 "P2" H 1350 2750 50  0000 C CNN
F 1 "LOOP1" V 1450 2550 50  0000 C CNN
F 2 "Connectors:bornier3" H 1350 2550 50  0001 C CNN
F 3 "" H 1350 2550 50  0000 C CNN
	1    1350 2550
	-1   0    0    1   
$EndComp
$Comp
L C_Small C5
U 1 1 5884A6F5
P 3000 3350
F 0 "C5" H 3010 3420 50  0000 L CNN
F 1 "100n" H 3010 3270 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3000 3350 50  0001 C CNN
F 3 "" H 3000 3350 50  0000 C CNN
	1    3000 3350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR016
U 1 1 5884A6FC
P 3000 3550
F 0 "#PWR016" H 3000 3300 50  0001 C CNN
F 1 "GND" H 3000 3400 50  0000 C CNN
F 2 "" H 3000 3550 50  0000 C CNN
F 3 "" H 3000 3550 50  0000 C CNN
	1    3000 3550
	1    0    0    -1  
$EndComp
$Comp
L C_Small C6
U 1 1 5884A702
P 3200 3350
F 0 "C6" H 3210 3420 50  0000 L CNN
F 1 "47n" H 3210 3270 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3200 3350 50  0001 C CNN
F 3 "" H 3200 3350 50  0000 C CNN
	1    3200 3350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 5884A709
P 3200 3550
F 0 "#PWR017" H 3200 3300 50  0001 C CNN
F 1 "GND" H 3200 3400 50  0000 C CNN
F 2 "" H 3200 3550 50  0000 C CNN
F 3 "" H 3200 3550 50  0000 C CNN
	1    3200 3550
	1    0    0    -1  
$EndComp
$Comp
L C_Small C7
U 1 1 5884A70F
P 3400 3350
F 0 "C7" H 3410 3420 50  0000 L CNN
F 1 "33n" H 3410 3270 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3400 3350 50  0001 C CNN
F 3 "" H 3400 3350 50  0000 C CNN
	1    3400 3350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR018
U 1 1 5884A716
P 3400 3550
F 0 "#PWR018" H 3400 3300 50  0001 C CNN
F 1 "GND" H 3400 3400 50  0000 C CNN
F 2 "" H 3400 3550 50  0000 C CNN
F 3 "" H 3400 3550 50  0000 C CNN
	1    3400 3550
	1    0    0    -1  
$EndComp
$Comp
L C_Small C8
U 1 1 5884A71C
P 3600 3350
F 0 "C8" H 3610 3420 50  0000 L CNN
F 1 "22n" H 3610 3270 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3600 3350 50  0001 C CNN
F 3 "" H 3600 3350 50  0000 C CNN
	1    3600 3350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR019
U 1 1 5884A723
P 3600 3550
F 0 "#PWR019" H 3600 3300 50  0001 C CNN
F 1 "GND" H 3600 3400 50  0000 C CNN
F 2 "" H 3600 3550 50  0000 C CNN
F 3 "" H 3600 3550 50  0000 C CNN
	1    3600 3550
	1    0    0    -1  
$EndComp
$Comp
L R_Small R6
U 1 1 5884A729
P 3650 2750
F 0 "R6" V 3550 2600 50  0000 L CNN
F 1 "1k" V 3550 2750 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 3650 2750 50  0001 C CNN
F 3 "" H 3650 2750 50  0000 C CNN
	1    3650 2750
	0    1    1    0   
$EndComp
$Comp
L BSS138 Q1
U 1 1 5884A730
P 4000 2700
F 0 "Q1" H 3850 2850 50  0000 L CNN
F 1 "BSS138" H 3800 2450 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 4200 2625 50  0001 L CIN
F 3 "" H 4000 2700 50  0000 L CNN
	1    4000 2700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR020
U 1 1 5884A737
P 4100 3550
F 0 "#PWR020" H 4100 3300 50  0001 C CNN
F 1 "GND" H 4100 3400 50  0000 C CNN
F 2 "" H 4100 3550 50  0000 C CNN
F 3 "" H 4100 3550 50  0000 C CNN
	1    4100 3550
	1    0    0    -1  
$EndComp
$Comp
L R_Small R2
U 1 1 5884A73D
P 4100 1750
F 0 "R2" V 4000 1600 50  0000 L CNN
F 1 "150" V 4000 1750 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 4100 1750 50  0001 C CNN
F 3 "" H 4100 1750 50  0000 C CNN
	1    4100 1750
	1    0    0    -1  
$EndComp
$Comp
L R_Small R1
U 1 1 5884A744
P 3900 1750
F 0 "R1" V 3800 1600 50  0000 L CNN
F 1 "150" V 3800 1750 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 3900 1750 50  0001 C CNN
F 3 "" H 3900 1750 50  0000 C CNN
	1    3900 1750
	1    0    0    -1  
$EndComp
$Comp
L BC817-40 Q2
U 1 1 5884A74B
P 4900 2950
F 0 "Q2" H 4750 3100 50  0000 L CNN
F 1 "BC817-40" H 4550 2700 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 5100 2875 50  0001 L CIN
F 3 "" H 4900 2950 50  0000 L CNN
	1    4900 2950
	1    0    0    -1  
$EndComp
$Comp
L R_Small R5
U 1 1 5884A752
P 4700 2600
F 0 "R5" V 4600 2450 50  0000 L CNN
F 1 "4.7k" V 4600 2600 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 4700 2600 50  0001 C CNN
F 3 "" H 4700 2600 50  0000 C CNN
	1    4700 2600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR021
U 1 1 5884A759
P 5000 3550
F 0 "#PWR021" H 5000 3300 50  0001 C CNN
F 1 "GND" H 5000 3400 50  0000 C CNN
F 2 "" H 5000 3550 50  0000 C CNN
F 3 "" H 5000 3550 50  0000 C CNN
	1    5000 3550
	1    0    0    -1  
$EndComp
$Comp
L R_Small R3
U 1 1 5884A75F
P 5000 1750
F 0 "R3" V 4900 1600 50  0000 L CNN
F 1 "1k" V 4900 1800 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 5000 1750 50  0001 C CNN
F 3 "" H 5000 1750 50  0000 C CNN
	1    5000 1750
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR022
U 1 1 5884A766
P 4100 1350
F 0 "#PWR022" H 4100 1200 50  0001 C CNN
F 1 "+5V" H 4100 1490 50  0000 C CNN
F 2 "" H 4100 1350 50  0000 C CNN
F 3 "" H 4100 1350 50  0000 C CNN
	1    4100 1350
	1    0    0    -1  
$EndComp
$Comp
L C_Small C4
U 1 1 5884A76C
P 4350 3300
F 0 "C4" H 4360 3370 50  0000 L CNN
F 1 "1u" H 4360 3220 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 4350 3300 50  0001 C CNN
F 3 "" H 4350 3300 50  0000 C CNN
	1    4350 3300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR023
U 1 1 5884A773
P 4350 3550
F 0 "#PWR023" H 4350 3300 50  0001 C CNN
F 1 "GND" H 4350 3400 50  0000 C CNN
F 2 "" H 4350 3550 50  0000 C CNN
F 3 "" H 4350 3550 50  0000 C CNN
	1    4350 3550
	1    0    0    -1  
$EndComp
$Comp
L R_Small R4
U 1 1 5884A779
P 5650 2450
F 0 "R4" V 5750 2450 50  0000 L CNN
F 1 "4.7k" V 5750 2250 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 5650 2450 50  0001 C CNN
F 3 "" H 5650 2450 50  0000 C CNN
	1    5650 2450
	0    1    1    0   
$EndComp
Text HLabel 5950 2450 2    60   Input ~ 0
osc2
$Comp
L D_Zener_Small ZD1
U 1 1 5884A7E8
P 3200 2250
F 0 "ZD1" H 3200 2340 50  0000 C CNN
F 1 "24V" H 3200 2160 50  0000 C CNN
F 2 "Diodes_SMD:D_MiniMELF_Standard" V 3200 2250 50  0001 C CNN
F 3 "" V 3200 2250 50  0000 C CNN
	1    3200 2250
	0    -1   -1   0   
$EndComp
$Comp
L D_Zener_Small ZD2
U 1 1 5884A7EF
P 3200 2550
F 0 "ZD2" H 3200 2640 50  0000 C CNN
F 1 "24V" H 3200 2460 50  0000 C CNN
F 2 "Diodes_SMD:D_MiniMELF_Standard" V 3200 2550 50  0001 C CNN
F 3 "" V 3200 2550 50  0000 C CNN
	1    3200 2550
	0    1    1    0   
$EndComp
$Comp
L JP3x2 JP2
U 1 1 588475D9
P 3300 6150
F 0 "JP2" H 3350 6300 60  0000 C CNN
F 1 "JP3x2" H 3350 5850 60  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x03_Pitch2.54mm" H 3300 6150 60  0001 C CNN
F 3 "" H 3300 6150 60  0000 C CNN
	1    3300 6150
	0    -1   -1   0   
$EndComp
$Comp
L JP3x2 JP1
U 1 1 5884A7C1
P 3300 3050
F 0 "JP1" H 3350 3200 60  0000 C CNN
F 1 "JP3x2" H 3350 2750 60  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x03_Pitch2.54mm" H 3300 3050 60  0001 C CNN
F 3 "" H 3300 3050 60  0000 C CNN
	1    3300 3050
	0    -1   -1   0   
$EndComp
$Comp
L +3.3V #PWR024
U 1 1 588593ED
P 5000 1350
F 0 "#PWR024" H 5000 1200 50  0001 C CNN
F 1 "+3.3V" H 5000 1490 50  0000 C CNN
F 2 "" H 5000 1350 50  0000 C CNN
F 3 "" H 5000 1350 50  0000 C CNN
	1    5000 1350
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR025
U 1 1 5885A643
P 5000 4450
F 0 "#PWR025" H 5000 4300 50  0001 C CNN
F 1 "+3.3V" H 5000 4590 50  0000 C CNN
F 2 "" H 5000 4450 50  0000 C CNN
F 3 "" H 5000 4450 50  0000 C CNN
	1    5000 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 5850 2100 5750
Wire Wire Line
	2100 5350 2100 5550
Wire Wire Line
	1550 5850 1550 5750
Wire Wire Line
	1550 5550 1550 5450
Wire Wire Line
	1550 5650 1650 5650
Wire Wire Line
	3000 6550 3000 6650
Wire Wire Line
	3200 6550 3200 6650
Wire Wire Line
	3400 6550 3400 6650
Wire Wire Line
	3600 6550 3600 6650
Wire Wire Line
	3000 5150 3000 6350
Wire Wire Line
	3000 5150 4700 5150
Wire Wire Line
	3200 5150 3200 5250
Wire Wire Line
	3200 5450 3200 5550
Wire Wire Line
	3750 5850 3800 5850
Wire Wire Line
	4100 6650 4100 6000
Wire Wire Line
	3900 4750 3900 4650
Wire Wire Line
	4700 5800 4700 6050
Wire Wire Line
	5000 6650 5000 6250
Wire Wire Line
	5000 4950 5000 5850
Wire Wire Line
	4100 4950 4100 5600
Connection ~ 3200 5150
Wire Wire Line
	3900 4950 3900 5150
Connection ~ 3900 5150
Connection ~ 4100 5150
Wire Wire Line
	4700 5150 4700 5600
Wire Wire Line
	5000 5550 5550 5550
Connection ~ 5000 5550
Wire Wire Line
	5950 5550 5750 5550
Wire Wire Line
	2900 5850 3550 5850
Wire Wire Line
	1550 5850 2300 5850
Wire Wire Line
	1550 5450 2300 5450
Connection ~ 2100 5450
Connection ~ 3000 5450
Connection ~ 2100 5850
Wire Wire Line
	3200 5750 3200 5850
Connection ~ 3200 5850
Wire Wire Line
	3200 6350 3200 6250
Wire Wire Line
	3200 6250 3300 6250
Wire Wire Line
	3400 6350 3400 6250
Wire Wire Line
	3600 6350 3600 6250
Wire Wire Line
	3600 6250 3500 6250
Wire Wire Line
	3300 5950 3300 5850
Connection ~ 3300 5850
Wire Wire Line
	3400 5950 3400 5850
Connection ~ 3400 5850
Wire Wire Line
	3500 5950 3500 5850
Connection ~ 3500 5850
Wire Wire Line
	4100 4450 4100 4750
Wire Wire Line
	2100 2750 2100 2650
Wire Wire Line
	2100 2250 2100 2450
Wire Wire Line
	1550 2750 1550 2650
Wire Wire Line
	1550 2450 1550 2350
Wire Wire Line
	1550 2550 1650 2550
Wire Wire Line
	3000 3450 3000 3550
Wire Wire Line
	3200 3450 3200 3550
Wire Wire Line
	3400 3450 3400 3550
Wire Wire Line
	3600 3450 3600 3550
Wire Wire Line
	3000 2050 3000 3250
Wire Wire Line
	3000 2050 4700 2050
Wire Wire Line
	3200 2050 3200 2150
Wire Wire Line
	3200 2350 3200 2450
Wire Wire Line
	3750 2750 3800 2750
Wire Wire Line
	4100 3550 4100 2900
Wire Wire Line
	3900 1650 3900 1550
Wire Wire Line
	4700 2700 4700 2950
Wire Wire Line
	5000 3550 5000 3150
Wire Wire Line
	5000 1850 5000 2750
Wire Wire Line
	4100 1850 4100 2500
Connection ~ 3200 2050
Wire Wire Line
	3900 1850 3900 2050
Connection ~ 3900 2050
Connection ~ 4100 2050
Wire Wire Line
	4700 2050 4700 2500
Wire Wire Line
	5000 2450 5550 2450
Connection ~ 5000 2450
Wire Wire Line
	5950 2450 5750 2450
Connection ~ 2100 2350
Connection ~ 2100 2750
Wire Wire Line
	3200 2650 3200 2750
Wire Wire Line
	3200 3250 3200 3150
Wire Wire Line
	3200 3150 3300 3150
Wire Wire Line
	3400 3250 3400 3150
Wire Wire Line
	3600 3250 3600 3150
Wire Wire Line
	3600 3150 3500 3150
Wire Wire Line
	3300 2750 3300 2850
Connection ~ 3300 2750
Wire Wire Line
	3400 2750 3400 2850
Connection ~ 3400 2750
Wire Wire Line
	3500 2750 3500 2850
Connection ~ 3500 2750
Wire Wire Line
	5000 1350 5000 1650
Wire Wire Line
	4100 1350 4100 1650
Wire Wire Line
	3900 1550 4350 1550
Connection ~ 4100 1550
Wire Wire Line
	4350 3400 4350 3550
Wire Wire Line
	4350 1550 4350 3200
Wire Wire Line
	3900 4650 4350 4650
Connection ~ 4100 4650
Wire Wire Line
	4350 6650 4350 6250
Wire Wire Line
	4350 4650 4350 6050
Wire Wire Line
	5000 4750 5000 4450
Wire Wire Line
	1850 2250 1850 2750
Connection ~ 1850 2750
Wire Wire Line
	1850 5350 1850 5850
Connection ~ 1850 5850
Wire Wire Line
	2100 5050 2100 5150
Wire Wire Line
	1650 5050 2100 5050
Wire Wire Line
	1650 5650 1650 5050
Wire Wire Line
	1850 5150 1850 5050
Connection ~ 1850 5050
Wire Wire Line
	2100 1950 2100 2050
Wire Wire Line
	1650 1950 2100 1950
Wire Wire Line
	1650 2550 1650 1950
Wire Wire Line
	1850 2050 1850 1950
Connection ~ 1850 1950
Connection ~ 3000 2350
Connection ~ 3200 2750
Wire Wire Line
	2900 2750 3550 2750
Wire Wire Line
	2900 2350 3000 2350
Wire Wire Line
	2900 5450 3000 5450
Wire Wire Line
	1550 2350 2300 2350
Wire Wire Line
	1550 2750 2300 2750
$EndSCHEMATC
