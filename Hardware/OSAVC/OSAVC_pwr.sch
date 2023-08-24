EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 4
Title "Power"
Date "2021-09-23"
Rev "1.0"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L power:+BATT #PWR0101
U 1 1 613A14DA
P 1400 1600
F 0 "#PWR0101" H 1400 1450 50  0001 C CNN
F 1 "+BATT" H 1415 1773 50  0000 C CNN
F 2 "" H 1400 1600 50  0001 C CNN
F 3 "" H 1400 1600 50  0001 C CNN
	1    1400 1600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 613A4C80
P 1400 2150
F 0 "#PWR0102" H 1400 1900 50  0001 C CNN
F 1 "GND" H 1405 1977 50  0000 C CNN
F 2 "" H 1400 2150 50  0001 C CNN
F 3 "" H 1400 2150 50  0001 C CNN
	1    1400 2150
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 613A4DBD
P 1550 1600
F 0 "#FLG0101" H 1550 1675 50  0001 C CNN
F 1 "PWR_FLAG" H 1550 1850 50  0000 C CNN
F 2 "" H 1550 1600 50  0001 C CNN
F 3 "~" H 1550 1600 50  0001 C CNN
	1    1550 1600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 613A5348
P 4100 2200
F 0 "#PWR0103" H 4100 1950 50  0001 C CNN
F 1 "GND" H 4105 2027 50  0000 C CNN
F 2 "" H 4100 2200 50  0001 C CNN
F 3 "" H 4100 2200 50  0001 C CNN
	1    4100 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 2000 4100 2100
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 613A5A32
P 1550 2150
F 0 "#FLG0102" H 1550 2225 50  0001 C CNN
F 1 "PWR_FLAG" H 1550 2400 50  0000 C CNN
F 2 "" H 1550 2150 50  0001 C CNN
F 3 "~" H 1550 2150 50  0001 C CNN
	1    1550 2150
	-1   0    0    1   
$EndComp
Wire Wire Line
	1550 1600 1400 1600
Wire Wire Line
	1400 2150 1550 2150
Wire Wire Line
	1400 1950 1400 2150
Connection ~ 1400 2150
Wire Wire Line
	1400 1850 1400 1650
Connection ~ 1400 1600
$Comp
L power:+BATT #PWR0104
U 1 1 613A7317
P 3500 1600
F 0 "#PWR0104" H 3500 1450 50  0001 C CNN
F 1 "+BATT" H 3515 1773 50  0000 C CNN
F 2 "" H 3500 1600 50  0001 C CNN
F 3 "" H 3500 1600 50  0001 C CNN
	1    3500 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 1600 3600 1600
$Comp
L Device:C C1
U 1 1 613A7C19
P 3500 1950
F 0 "C1" H 3250 1950 50  0000 L CNN
F 1 "2.2uF" H 3250 1850 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3538 1800 50  0001 C CNN
F 3 "~" H 3500 1950 50  0001 C CNN
	1    3500 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 1800 3500 1600
Connection ~ 3500 1600
$Comp
L power:GND #PWR0105
U 1 1 613A81E6
P 3500 2200
F 0 "#PWR0105" H 3500 1950 50  0001 C CNN
F 1 "GND" H 3505 2027 50  0000 C CNN
F 2 "" H 3500 2200 50  0001 C CNN
F 3 "" H 3500 2200 50  0001 C CNN
	1    3500 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 2200 3500 2100
$Comp
L Device:R_US R1
U 1 1 613AD6F3
P 4700 1950
F 0 "R1" H 4768 1996 50  0000 L CNN
F 1 "8.76K " H 4768 1905 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4740 1940 50  0001 C CNN
F 3 "~" H 4700 1950 50  0001 C CNN
	1    4700 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 1800 4600 1800
NoConn ~ 4600 1700
NoConn ~ 3600 1700
$Comp
L Device:CP1 C2
U 1 1 613AEE11
P 5100 1950
F 0 "C2" H 5215 1996 50  0000 L CNN
F 1 "330uF" H 5215 1905 50  0000 L CNN
F 2 "OSAVC:EEE-FNA331XUL" H 5100 1950 50  0001 C CNN
F 3 "~" H 5100 1950 50  0001 C CNN
	1    5100 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 1800 5100 1600
$Comp
L power:GND #PWR0106
U 1 1 613AFB34
P 5100 2200
F 0 "#PWR0106" H 5100 1950 50  0001 C CNN
F 1 "GND" H 5105 2027 50  0000 C CNN
F 2 "" H 5100 2200 50  0001 C CNN
F 3 "" H 5100 2200 50  0001 C CNN
	1    5100 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 1600 4600 1600
Text GLabel 5200 1600 2    50   Input ~ 0
+7V
Wire Wire Line
	4700 2100 4100 2100
Connection ~ 4100 2100
Wire Wire Line
	4100 2100 4100 2200
$Comp
L LPC38690DT-33:LP38690DT-3.3_NOPB U3
U 1 1 613C0416
P 4850 6600
F 0 "U3" H 5650 6113 60  0000 C CNN
F 1 "LP38690DT-3.3_NOPB" H 5650 6219 60  0000 C CNN
F 2 "LPC38690DT:LP38690DT-3.3&slash_NOPB" H 5650 6840 60  0001 C CNN
F 3 "" H 4850 6600 60  0000 C CNN
	1    4850 6600
	-1   0    0    1   
$EndComp
$Comp
L Device:C C3
U 1 1 613C3C14
P 3150 5050
F 0 "C3" H 2900 5050 50  0000 L CNN
F 1 "10uF" H 2900 4950 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 3188 4900 50  0001 C CNN
F 3 "~" H 3150 5050 50  0001 C CNN
	1    3150 5050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 613C5F96
P 4900 5150
F 0 "C4" H 5015 5196 50  0000 L CNN
F 1 "10uF" H 5015 5105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 4938 5000 50  0001 C CNN
F 3 "~" H 4900 5150 50  0001 C CNN
	1    4900 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 613C6982
P 4850 6800
F 0 "C6" H 4965 6846 50  0000 L CNN
F 1 "10uF" H 4965 6755 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 4888 6650 50  0001 C CNN
F 3 "~" H 4850 6800 50  0001 C CNN
	1    4850 6800
	1    0    0    -1  
$EndComp
Text GLabel 3050 4900 0    50   Input ~ 0
+7V
$Comp
L power:GND #PWR0107
U 1 1 613C7BB4
P 3150 5300
F 0 "#PWR0107" H 3150 5050 50  0001 C CNN
F 1 "GND" H 3155 5127 50  0000 C CNN
F 2 "" H 3150 5300 50  0001 C CNN
F 3 "" H 3150 5300 50  0001 C CNN
	1    3150 5300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 613C8406
P 4900 5350
F 0 "#PWR0108" H 4900 5100 50  0001 C CNN
F 1 "GND" H 4905 5177 50  0000 C CNN
F 2 "" H 4900 5350 50  0001 C CNN
F 3 "" H 4900 5350 50  0001 C CNN
	1    4900 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 5350 4900 5300
$Comp
L power:GND #PWR0110
U 1 1 613C9928
P 4850 7000
F 0 "#PWR0110" H 4850 6750 50  0001 C CNN
F 1 "GND" H 4855 6827 50  0000 C CNN
F 2 "" H 4850 7000 50  0001 C CNN
F 3 "" H 4850 7000 50  0001 C CNN
	1    4850 7000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 613C9D9D
P 3100 6900
F 0 "#PWR0111" H 3100 6650 50  0001 C CNN
F 1 "GND" H 3105 6727 50  0000 C CNN
F 2 "" H 3100 6900 50  0001 C CNN
F 3 "" H 3100 6900 50  0001 C CNN
	1    3100 6900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 6950 4850 7000
Wire Wire Line
	3100 6800 3100 6900
$Comp
L power:+5V #PWR0113
U 1 1 613CCD59
P 4900 4900
F 0 "#PWR0113" H 4900 4750 50  0001 C CNN
F 1 "+5V" H 4915 5073 50  0000 C CNN
F 2 "" H 4900 4900 50  0001 C CNN
F 3 "" H 4900 4900 50  0001 C CNN
	1    4900 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 5000 4900 4900
$Comp
L power:+3V3 #PWR0114
U 1 1 613CDFB4
P 4850 6500
F 0 "#PWR0114" H 4850 6350 50  0001 C CNN
F 1 "+3V3" H 4865 6673 50  0000 C CNN
F 2 "" H 4850 6500 50  0001 C CNN
F 3 "" H 4850 6500 50  0001 C CNN
	1    4850 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 6500 4850 6600
Connection ~ 4850 6600
Wire Wire Line
	4850 6600 4850 6650
$Comp
L LPC38690DT-5:LP38690DTX-5.0_NOPB U2
U 1 1 613BFDA8
P 4900 5000
F 0 "U2" H 5700 4513 60  0000 C CNN
F 1 "LP38690DTX-5.0_NOPB" H 5700 4619 60  0000 C CNN
F 2 "LPC38690DT:LP38690DTX-5.0&slash_NOPB" H 5700 5240 60  0001 C CNN
F 3 "" H 4900 5000 60  0000 C CNN
	1    4900 5000
	-1   0    0    1   
$EndComp
Wire Wire Line
	3300 4900 3150 4900
Wire Wire Line
	3300 5000 3300 5200
Wire Wire Line
	3300 5200 3150 5200
Connection ~ 3150 5200
Wire Wire Line
	3150 5200 3150 5300
Connection ~ 3150 4900
Wire Wire Line
	3150 4900 3050 4900
Connection ~ 4900 5000
$Comp
L Device:C C5
U 1 1 613C3FA6
P 3100 6650
F 0 "C5" H 2800 6650 50  0000 L CNN
F 1 "10uF" H 2800 6550 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 3138 6500 50  0001 C CNN
F 3 "~" H 3100 6650 50  0001 C CNN
	1    3100 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 6500 3250 6500
Wire Wire Line
	3250 6600 3250 6800
Wire Wire Line
	3250 6800 3100 6800
Connection ~ 3100 6800
$Comp
L Device:R_US R3
U 1 1 613F1516
P 7000 2100
F 0 "R3" H 7068 2146 50  0000 L CNN
F 1 "10K" H 7068 2055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7040 2090 50  0001 C CNN
F 3 "~" H 7000 2100 50  0001 C CNN
	1    7000 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R5
U 1 1 613F17E1
P 7000 2550
F 0 "R5" H 7068 2596 50  0000 L CNN
F 1 "10K" H 7068 2505 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7040 2540 50  0001 C CNN
F 3 "~" H 7000 2550 50  0001 C CNN
	1    7000 2550
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:FDN340P Q1
U 1 1 613F2F1B
P 9250 1650
F 0 "Q1" V 9592 1650 50  0000 C CNN
F 1 "AO3423" V 9501 1650 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 9450 1575 50  0001 L CIN
F 3 "http://aosmd.com/res/data_sheets/AO3423.pdf" H 9250 1650 50  0001 L CNN
	1    9250 1650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7000 2250 7000 2350
Wire Wire Line
	7000 2350 8050 2350
Connection ~ 7000 2350
Wire Wire Line
	7000 2350 7000 2400
$Comp
L power:GND #PWR0109
U 1 1 613F6D4B
P 7000 2800
F 0 "#PWR0109" H 7000 2550 50  0001 C CNN
F 1 "GND" H 7005 2627 50  0000 C CNN
F 2 "" H 7000 2800 50  0001 C CNN
F 3 "" H 7000 2800 50  0001 C CNN
	1    7000 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 2800 7000 2700
$Comp
L power:GND #PWR0112
U 1 1 613F7AB6
P 8250 2850
F 0 "#PWR0112" H 8250 2600 50  0001 C CNN
F 1 "GND" H 8255 2677 50  0000 C CNN
F 2 "" H 8250 2850 50  0001 C CNN
F 3 "" H 8250 2850 50  0001 C CNN
	1    8250 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 2850 8250 2750
$Comp
L power:+5V #PWR0115
U 1 1 613F8605
P 8250 2050
F 0 "#PWR0115" H 8250 1900 50  0001 C CNN
F 1 "+5V" H 8265 2223 50  0000 C CNN
F 2 "" H 8250 2050 50  0001 C CNN
F 3 "" H 8250 2050 50  0001 C CNN
	1    8250 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 2050 8250 2150
$Comp
L power:+3V3 #PWR0117
U 1 1 613FA08D
P 7900 2550
F 0 "#PWR0117" H 7900 2400 50  0001 C CNN
F 1 "+3V3" H 7750 2550 50  0000 C CNN
F 2 "" H 7900 2550 50  0001 C CNN
F 3 "" H 7900 2550 50  0001 C CNN
	1    7900 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 2550 8050 2550
Text GLabel 8900 1550 0    50   Input ~ 0
USB5V
Wire Wire Line
	8900 1550 9050 1550
Wire Wire Line
	8650 2450 9250 2450
Wire Wire Line
	9250 2450 9250 1850
$Comp
L power:+5V #PWR0118
U 1 1 613FD18D
P 9600 1550
F 0 "#PWR0118" H 9600 1400 50  0001 C CNN
F 1 "+5V" H 9615 1723 50  0000 C CNN
F 2 "" H 9600 1550 50  0001 C CNN
F 3 "" H 9600 1550 50  0001 C CNN
	1    9600 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 1550 9450 1550
$Comp
L Device:R_US R7
U 1 1 6142F1A4
P 5650 6800
F 0 "R7" H 5718 6846 50  0000 L CNN
F 1 "470" H 5718 6755 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5690 6790 50  0001 C CNN
F 3 "~" H 5650 6800 50  0001 C CNN
	1    5650 6800
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D2
U 1 1 614300C5
P 5650 7150
F 0 "D2" V 5689 7032 50  0000 R CNN
F 1 "LED" V 5598 7032 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 5650 7150 50  0001 C CNN
F 3 "~" H 5650 7150 50  0001 C CNN
	1    5650 7150
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 61435558
P 5650 7350
F 0 "#PWR0119" H 5650 7100 50  0001 C CNN
F 1 "GND" H 5655 7177 50  0000 C CNN
F 2 "" H 5650 7350 50  0001 C CNN
F 3 "" H 5650 7350 50  0001 C CNN
	1    5650 7350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 7350 5650 7300
$Comp
L Device:R_US R2
U 1 1 61436386
P 2100 1800
F 0 "R2" H 2168 1846 50  0000 L CNN
F 1 "22K" H 2168 1755 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2140 1790 50  0001 C CNN
F 3 "~" H 2100 1800 50  0001 C CNN
	1    2100 1800
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R4
U 1 1 6143698C
P 2100 2150
F 0 "R4" H 2168 2196 50  0000 L CNN
F 1 "2K" H 2168 2105 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2140 2140 50  0001 C CNN
F 3 "~" H 2100 2150 50  0001 C CNN
	1    2100 2150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0120
U 1 1 614372D1
P 2100 2350
F 0 "#PWR0120" H 2100 2100 50  0001 C CNN
F 1 "GND" H 2105 2177 50  0000 C CNN
F 2 "" H 2100 2350 50  0001 C CNN
F 3 "" H 2100 2350 50  0001 C CNN
	1    2100 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 1650 1400 1650
Connection ~ 1400 1650
Wire Wire Line
	1400 1650 1400 1600
Wire Wire Line
	2100 1950 2100 2000
Wire Wire Line
	2100 2300 2100 2350
$Comp
L power:+5V #PWR0122
U 1 1 614848EA
P 3100 6450
F 0 "#PWR0122" H 3100 6300 50  0001 C CNN
F 1 "+5V" H 3115 6623 50  0000 C CNN
F 2 "" H 3100 6450 50  0001 C CNN
F 3 "" H 3100 6450 50  0001 C CNN
	1    3100 6450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 6450 3100 6500
Connection ~ 3100 6500
Text Notes 1650 1200 0    100  ~ 0
Battery input
Text Notes 3700 1200 0    100  ~ 0
Main supply
Text Notes 3550 2750 0    100  ~ 0
Servo and CC 5V
Text Notes 3550 4350 0    100  ~ 0
Main 5V supply
Text Notes 3450 6000 0    100  ~ 0
Main 3.3V supply
Text Notes 7700 1250 0    100  ~ 0
USB Power switch
Wire Wire Line
	5650 6600 5650 6650
Wire Wire Line
	5650 6950 5650 7000
Text GLabel 2400 1950 2    50   Input ~ 0
AN0
$Comp
L Device:C_Small C26
U 1 1 6156D01C
P 5300 5150
F 0 "C26" H 5392 5196 50  0000 L CNN
F 1 "0.1uF" H 5392 5105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 5300 5150 50  0001 C CNN
F 3 "~" H 5300 5150 50  0001 C CNN
	1    5300 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 5050 5300 5000
Wire Wire Line
	5300 5000 4900 5000
Wire Wire Line
	5300 5250 5300 5350
Wire Wire Line
	5300 5350 4900 5350
Connection ~ 4900 5350
Wire Wire Line
	5100 2100 5100 2200
Wire Wire Line
	5200 1600 5100 1600
Connection ~ 5100 1600
$Comp
L Device:C_Small C27
U 1 1 61576DFF
P 5250 6800
F 0 "C27" H 5342 6846 50  0000 L CNN
F 1 "0.1uF" H 5342 6755 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 5250 6800 50  0001 C CNN
F 3 "~" H 5250 6800 50  0001 C CNN
	1    5250 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 6600 5250 6600
Wire Wire Line
	5250 6700 5250 6600
Wire Wire Line
	5250 6900 5250 6950
Wire Wire Line
	5250 6950 4850 6950
Connection ~ 4850 6950
Wire Wire Line
	5250 6600 5650 6600
Connection ~ 5250 6600
Text GLabel 6950 1800 0    50   Input ~ 0
+7V
Wire Wire Line
	6950 1800 7000 1800
Wire Wire Line
	7000 1800 7000 1950
Wire Wire Line
	1400 1950 1350 1950
Wire Wire Line
	1400 1850 1350 1850
Wire Wire Line
	2400 1950 2100 1950
Connection ~ 2100 1950
$Comp
L Device:C C8
U 1 1 61624439
P 5800 3650
F 0 "C8" H 5915 3696 50  0000 L CNN
F 1 "10uF" H 5915 3605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 5838 3500 50  0001 C CNN
F 3 "~" H 5800 3650 50  0001 C CNN
	1    5800 3650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 61624FA0
P 5800 3850
F 0 "#PWR0116" H 5800 3600 50  0001 C CNN
F 1 "GND" H 5805 3677 50  0000 C CNN
F 2 "" H 5800 3850 50  0001 C CNN
F 3 "" H 5800 3850 50  0001 C CNN
	1    5800 3850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0121
U 1 1 616292C3
P 5500 3300
F 0 "#PWR0121" H 5500 3050 50  0001 C CNN
F 1 "GND" H 5500 3350 50  0000 C CNN
F 2 "" H 5500 3300 50  0001 C CNN
F 3 "" H 5500 3300 50  0001 C CNN
	1    5500 3300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0123
U 1 1 6162B8F4
P 2950 3600
F 0 "#PWR0123" H 2950 3350 50  0001 C CNN
F 1 "GND" H 2955 3427 50  0000 C CNN
F 2 "" H 2950 3600 50  0001 C CNN
F 3 "" H 2950 3600 50  0001 C CNN
	1    2950 3600
	1    0    0    -1  
$EndComp
Text GLabel 2850 3350 0    50   Input ~ 0
+7V
$Comp
L power:+5VA #PWR0124
U 1 1 616375E3
P 5800 3450
F 0 "#PWR0124" H 5800 3300 50  0001 C CNN
F 1 "+5VA" H 5815 3623 50  0000 C CNN
F 2 "" H 5800 3450 50  0001 C CNN
F 3 "" H 5800 3450 50  0001 C CNN
	1    5800 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 3450 5800 3450
Wire Wire Line
	5800 3450 5800 3500
Connection ~ 5800 3450
$Comp
L MIC293000:MIC29300-5.0WU U5
U 1 1 61621E19
P 3200 3350
F 0 "U5" H 4300 3738 60  0000 C CNN
F 1 "MIC29300-5.0WU" H 4300 3632 60  0000 C CNN
F 2 "MIC293000:MIC29300-5.0WU" H 4300 3590 60  0001 C CNN
F 3 "" H 3200 3350 60  0000 C CNN
	1    3200 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 3300 5400 3300
Wire Wire Line
	5400 3300 5400 3350
Wire Wire Line
	5800 3800 5800 3850
$Comp
L Device:C_Small C25
U 1 1 61643FEA
P 6200 3650
F 0 "C25" H 6292 3696 50  0000 L CNN
F 1 "0.1uF" H 6292 3605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 6200 3650 50  0001 C CNN
F 3 "~" H 6200 3650 50  0001 C CNN
	1    6200 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 3450 6200 3450
Wire Wire Line
	6200 3450 6200 3550
Wire Wire Line
	6200 3750 6200 3800
Wire Wire Line
	6200 3800 5800 3800
Connection ~ 5800 3800
$Comp
L Device:C_Small C7
U 1 1 61647AC5
P 2950 3450
F 0 "C7" H 2750 3400 50  0000 L CNN
F 1 "0.1uF" H 2700 3300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 2950 3450 50  0001 C CNN
F 3 "~" H 2950 3450 50  0001 C CNN
	1    2950 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 3350 2950 3350
Connection ~ 2950 3350
Wire Wire Line
	2950 3600 2950 3550
Wire Wire Line
	2950 3350 3200 3350
Wire Wire Line
	3200 3450 3200 3550
Wire Wire Line
	3200 3550 2950 3550
Connection ~ 2950 3550
$Comp
L Amplifier_Operational:MCP6001U U4
U 1 1 6168A007
P 8350 2450
F 0 "U4" H 8694 2496 50  0000 L CNN
F 1 "MCP6001U" H 8694 2405 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 8350 2450 50  0001 L CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21733j.pdf" H 8350 2450 50  0001 C CNN
	1    8350 2450
	1    0    0    -1  
$EndComp
$Comp
L Converter_DCDC:PTN78020W_EUK-7 U1
U 1 1 6184A2FC
P 4100 1700
F 0 "U1" H 4100 2067 50  0000 C CNN
F 1 "PTN78020W_EUK-7" H 4100 1976 50  0000 C CNN
F 2 "Module:Texas_EUK_R-PDSS-T7_THT" H 4000 1300 50  0001 C CNN
F 3 "https://www.ti.com/lit/ds/symlink/ptn78020w.pdf" H 4075 1750 50  0001 C CNN
	1    4100 1700
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Female J1
U 1 1 61F8359D
P 1150 1850
F 0 "J1" H 1150 1650 50  0000 C CNN
F 1 "Battery In" H 1150 1950 50  0000 C CNN
F 2 "OSAVC:XT60" H 1150 1850 50  0001 C CNN
F 3 "~" H 1150 1850 50  0001 C CNN
	1    1150 1850
	-1   0    0    -1  
$EndComp
$EndSCHEMATC
