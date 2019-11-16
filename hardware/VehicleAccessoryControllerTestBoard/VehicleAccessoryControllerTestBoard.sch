EESchema Schematic File Version 4
LIBS:VehicleAccessoryControllerTestBoard-cache
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 1 1
Title "Vehicle Accessory Controller Test Board"
Date "2019-11-15"
Rev "0.0"
Comp "Noctivore"
Comment1 "IF IN DOUBT - ASK"
Comment2 "Designed by Ops"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector_Generic:Conn_01x03 J501
U 1 1 5DAF4564
P 1075 1250
F 0 "J501" H 1075 1450 50  0000 C CNN
F 1 "Input" V 1175 1250 50  0000 C CNN
F 2 "VehicleAccessoryController:Keystone_8719" H 1075 1250 50  0001 C CNN
F 3 "~" H 1075 1250 50  0001 C CNN
	1    1075 1250
	-1   0    0    1   
$EndComp
$Comp
L VehicleAccessoryController:FID fid1
U 1 1 5DAF80EF
P 7825 825
F 0 "fid1" H 7775 975 60  0000 L CNN
F 1 "FID" H 7775 675 60  0000 L CNN
F 2 "VehicleAccessoryController:Fiducial" H 7825 825 60  0001 C CNN
F 3 "" H 7825 825 60  0001 C CNN
	1    7825 825 
	1    0    0    -1  
$EndComp
$Comp
L VehicleAccessoryController:FID fid2
U 1 1 5DB12261
P 8175 825
F 0 "fid2" H 8125 975 60  0000 L CNN
F 1 "FID" H 8125 675 60  0000 L CNN
F 2 "VehicleAccessoryController:Fiducial" H 8175 825 60  0001 C CNN
F 3 "" H 8175 825 60  0001 C CNN
	1    8175 825 
	1    0    0    -1  
$EndComp
$Comp
L VehicleAccessoryController:FID fid3
U 1 1 5DB12492
P 8525 825
F 0 "fid3" H 8475 975 60  0000 L CNN
F 1 "FID" H 8475 675 60  0000 L CNN
F 2 "VehicleAccessoryController:Fiducial" H 8525 825 60  0001 C CNN
F 3 "" H 8525 825 60  0001 C CNN
	1    8525 825 
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x03 J502
U 1 1 5DB225D1
P 10175 1050
F 0 "J502" H 10050 1250 50  0000 L CNN
F 1 "Front Conn" V 10275 850 50  0000 L CNN
F 2 "VehicleAccessoryController:Keystone_8719" H 10175 1050 50  0001 C CNN
F 3 "~" H 10175 1050 50  0001 C CNN
	1    10175 1050
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x03 J503
U 1 1 5DB2275A
P 10175 2550
F 0 "J503" H 10050 2750 50  0000 L CNN
F 1 "Rear Conn" V 10275 2375 50  0000 L CNN
F 2 "VehicleAccessoryController:Keystone_8719" H 10175 2550 50  0001 C CNN
F 3 "~" H 10175 2550 50  0001 C CNN
	1    10175 2550
	1    0    0    -1  
$EndComp
$Comp
L power:GNDA #PWR02
U 1 1 5DB250EC
P 1800 1325
F 0 "#PWR02" H 1800 1075 50  0001 C CNN
F 1 "GNDA" H 1805 1152 50  0000 C CNN
F 2 "" H 1800 1325 50  0001 C CNN
F 3 "" H 1800 1325 50  0001 C CNN
	1    1800 1325
	1    0    0    -1  
$EndComp
$Comp
L power:+12VA #PWR01
U 1 1 5DB2511B
P 1425 1075
F 0 "#PWR01" H 1425 925 50  0001 C CNN
F 1 "+12VA" H 1440 1248 50  0000 C CNN
F 2 "" H 1425 1075 50  0001 C CNN
F 3 "" H 1425 1075 50  0001 C CNN
	1    1425 1075
	1    0    0    -1  
$EndComp
Wire Wire Line
	1275 1150 1425 1150
Wire Wire Line
	1425 1150 1425 1075
Wire Wire Line
	1275 1350 1425 1350
Text Label 1425 1350 0    50   ~ 0
ACC_IN
$Comp
L Device:LED_ALT D601
U 1 1 5DB32C02
P 1875 6900
F 0 "D601" H 1875 7025 50  0000 C CNN
F 1 "LED_ALT" H 1875 6750 50  0000 C CNN
F 2 "VehicleAccessoryController:LED_0603" H 1875 6900 50  0001 C CNN
F 3 "~" H 1875 6900 50  0001 C CNN
	1    1875 6900
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR05
U 1 1 5DB32CA0
P 2175 6825
F 0 "#PWR05" H 2175 6675 50  0001 C CNN
F 1 "+5V" H 2190 6998 50  0000 C CNN
F 2 "" H 2175 6825 50  0001 C CNN
F 3 "" H 2175 6825 50  0001 C CNN
	1    2175 6825
	1    0    0    -1  
$EndComp
$Comp
L Device:R R706
U 1 1 5DB32F81
P 1425 6900
F 0 "R706" V 1575 6800 50  0000 L CNN
F 1 "430R" V 1500 6800 50  0000 L CNN
F 2 "VehicleAccessoryController:R_0603" V 1355 6900 50  0001 C CNN
F 3 "~" H 1425 6900 50  0001 C CNN
	1    1425 6900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1575 6900 1725 6900
Wire Wire Line
	2025 6900 2175 6900
Wire Wire Line
	2175 6900 2175 6825
Wire Wire Line
	1125 6900 1275 6900
Text Label 1125 6900 2    50   ~ 0
PARK_STATUS
$Comp
L power:+5V #PWR0102
U 1 1 5DB396F4
P 9825 875
F 0 "#PWR0102" H 9825 725 50  0001 C CNN
F 1 "+5V" H 9840 1048 50  0000 C CNN
F 2 "" H 9825 875 50  0001 C CNN
F 3 "" H 9825 875 50  0001 C CNN
	1    9825 875 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 5DB397BD
P 9825 1850
F 0 "#PWR0110" H 9825 1600 50  0001 C CNN
F 1 "GND" H 9830 1677 50  0000 C CNN
F 2 "" H 9825 1850 50  0001 C CNN
F 3 "" H 9825 1850 50  0001 C CNN
	1    9825 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9825 875  9825 950 
Wire Wire Line
	9825 950  9975 950 
Wire Wire Line
	9975 1150 9825 1150
Text Label 8175 1575 2    50   ~ 0
PARK_FRONT
$Comp
L power:+5V #PWR0111
U 1 1 5DB3DD66
P 9825 2375
F 0 "#PWR0111" H 9825 2225 50  0001 C CNN
F 1 "+5V" H 9840 2548 50  0000 C CNN
F 2 "" H 9825 2375 50  0001 C CNN
F 3 "" H 9825 2375 50  0001 C CNN
	1    9825 2375
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 5DB3DD6C
P 9825 3350
F 0 "#PWR0112" H 9825 3100 50  0001 C CNN
F 1 "GND" H 9830 3177 50  0000 C CNN
F 2 "" H 9825 3350 50  0001 C CNN
F 3 "" H 9825 3350 50  0001 C CNN
	1    9825 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	9825 2375 9825 2450
Wire Wire Line
	9825 2450 9975 2450
Wire Wire Line
	9975 2650 9825 2650
$Comp
L Switch:SW_SPDT SW301
U 1 1 5DB3F551
P 3150 2725
F 0 "SW301" H 3150 2900 50  0000 C CNN
F 1 "ON/OFF" H 3150 2500 50  0000 C CNN
F 2 "VehicleAccessoryController:C&K_ET_Series_Switch" H 3150 2725 50  0001 C CNN
F 3 "" H 3150 2725 50  0001 C CNN
	1    3150 2725
	1    0    0    1   
$EndComp
$Comp
L VehicleAccessoryController:TPS3710 U103
U 1 1 5DB4FA4F
P 5450 1350
F 0 "U103" H 5150 1600 50  0000 C CNN
F 1 "TPS3710" H 5450 1150 50  0000 C CNN
F 2 "VehicleAccessoryController:TI DSE (S-PWSON-N6)" H 5450 1350 50  0001 C CNN
F 3 "" H 5450 1350 50  0001 C CNN
	1    5450 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R703
U 1 1 5DB51851
P 4550 1125
F 0 "R703" H 4625 1075 50  0000 L CNN
F 1 "2M43" H 4625 1175 50  0000 L CNN
F 2 "VehicleAccessoryController:R_0603" V 4480 1125 50  0001 C CNN
F 3 "~" H 4550 1125 50  0001 C CNN
	1    4550 1125
	-1   0    0    1   
$EndComp
$Comp
L Device:R R704
U 1 1 5DB51903
P 4550 1575
F 0 "R704" H 4625 1525 50  0000 L CNN
F 1 "84k5" H 4625 1625 50  0000 L CNN
F 2 "VehicleAccessoryController:R_0603" V 4480 1575 50  0001 C CNN
F 3 "~" H 4550 1575 50  0001 C CNN
	1    4550 1575
	-1   0    0    1   
$EndComp
$Comp
L Device:R R705
U 1 1 5DB5195B
P 2725 1300
F 0 "R705" H 2800 1250 50  0000 L CNN
F 1 "49k9" H 2800 1350 50  0000 L CNN
F 2 "VehicleAccessoryController:R_0603" V 2655 1300 50  0001 C CNN
F 3 "~" H 2725 1300 50  0001 C CNN
	1    2725 1300
	-1   0    0    1   
$EndComp
$Comp
L power:+12VA #PWR0117
U 1 1 5DB534DE
P 4850 1175
F 0 "#PWR0117" H 4850 1025 50  0001 C CNN
F 1 "+12VA" H 4865 1348 50  0000 C CNN
F 2 "" H 4850 1175 50  0001 C CNN
F 3 "" H 4850 1175 50  0001 C CNN
	1    4850 1175
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 1250 4850 1250
Wire Wire Line
	4850 1250 4850 1175
$Comp
L Device:C C804
U 1 1 5DB5500A
P 4100 1350
F 0 "C804" H 3775 1425 50  0000 L CNN
F 1 "0.1uF" H 3775 1325 50  0000 L CNN
F 2 "VehicleAccessoryController:C_0603" H 4138 1200 50  0001 C CNN
F 3 "~" H 4100 1350 50  0001 C CNN
	1    4100 1350
	1    0    0    -1  
$EndComp
$Comp
L power:GNDA #PWR0118
U 1 1 5DB551BA
P 4850 1525
F 0 "#PWR0118" H 4850 1275 50  0001 C CNN
F 1 "GNDA" H 4855 1352 50  0000 C CNN
F 2 "" H 4850 1525 50  0001 C CNN
F 3 "" H 4850 1525 50  0001 C CNN
	1    4850 1525
	1    0    0    -1  
$EndComp
$Comp
L power:GNDA #PWR0119
U 1 1 5DB5525B
P 6050 1525
F 0 "#PWR0119" H 6050 1275 50  0001 C CNN
F 1 "GNDA" H 6055 1352 50  0000 C CNN
F 2 "" H 6050 1525 50  0001 C CNN
F 3 "" H 6050 1525 50  0001 C CNN
	1    6050 1525
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 1350 6050 1350
Wire Wire Line
	6050 1350 6050 1450
Wire Wire Line
	5900 1450 6050 1450
Connection ~ 6050 1450
Wire Wire Line
	6050 1450 6050 1525
Wire Wire Line
	5000 1450 4850 1450
Wire Wire Line
	4850 1450 4850 1525
$Comp
L power:+12VA #PWR0120
U 1 1 5DB62781
P 4550 900
F 0 "#PWR0120" H 4550 750 50  0001 C CNN
F 1 "+12VA" H 4565 1073 50  0000 C CNN
F 2 "" H 4550 900 50  0001 C CNN
F 3 "" H 4550 900 50  0001 C CNN
	1    4550 900 
	1    0    0    -1  
$EndComp
$Comp
L power:GNDA #PWR0121
U 1 1 5DB627BE
P 4550 1800
F 0 "#PWR0121" H 4550 1550 50  0001 C CNN
F 1 "GNDA" H 4555 1627 50  0000 C CNN
F 2 "" H 4550 1800 50  0001 C CNN
F 3 "" H 4550 1800 50  0001 C CNN
	1    4550 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 975  4550 900 
Wire Wire Line
	4550 1425 4550 1350
Wire Wire Line
	4550 1800 4550 1725
Wire Wire Line
	5000 1350 4550 1350
Connection ~ 4550 1350
Wire Wire Line
	4550 1350 4550 1275
Wire Wire Line
	6050 1250 5900 1250
$Comp
L power:+12VA #PWR0122
U 1 1 5DB6F04B
P 4100 900
F 0 "#PWR0122" H 4100 750 50  0001 C CNN
F 1 "+12VA" H 4115 1073 50  0000 C CNN
F 2 "" H 4100 900 50  0001 C CNN
F 3 "" H 4100 900 50  0001 C CNN
	1    4100 900 
	1    0    0    -1  
$EndComp
$Comp
L power:GNDA #PWR0123
U 1 1 5DB6F088
P 4100 1800
F 0 "#PWR0123" H 4100 1550 50  0001 C CNN
F 1 "GNDA" H 4105 1627 50  0000 C CNN
F 2 "" H 4100 1800 50  0001 C CNN
F 3 "" H 4100 1800 50  0001 C CNN
	1    4100 1800
	1    0    0    -1  
$EndComp
Text Label 6050 1250 0    50   ~ 0
SHUTDOWN
Text Notes 6025 1150 0    50   ~ 0
(Active Low)
$Comp
L power:+12VA #PWR0124
U 1 1 5DB6F285
P 2725 1075
F 0 "#PWR0124" H 2725 925 50  0001 C CNN
F 1 "+12VA" H 2740 1248 50  0000 C CNN
F 2 "" H 2725 1075 50  0001 C CNN
F 3 "" H 2725 1075 50  0001 C CNN
	1    2725 1075
	1    0    0    -1  
$EndComp
Wire Wire Line
	2725 1600 2725 1450
Text Label 2725 1600 2    50   ~ 0
CTRL
Wire Wire Line
	4100 900  4100 1200
Wire Wire Line
	4100 1500 4100 1800
Text Notes 4950 2675 0    50   ~ 0
             |       R1  |\nVmon(uv) = | 1 + ----| x Vit-\n             |       R2  |\n\n             |      2M43|\nVmon(uv) = | 1 + ----| x 394.5mV\n             |      84k5 |\n\nVmon(uv) = 11.739V
$Comp
L Device:R R720
U 1 1 5DB687E6
P 9525 1200
F 0 "R720" H 9600 1150 50  0000 L CNN
F 1 "30k9" H 9600 1250 50  0000 L CNN
F 2 "VehicleAccessoryController:R_0603" V 9455 1200 50  0001 C CNN
F 3 "~" H 9525 1200 50  0001 C CNN
	1    9525 1200
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0125
U 1 1 5DB690C0
P 9525 1850
F 0 "#PWR0125" H 9525 1600 50  0001 C CNN
F 1 "GND" H 9530 1677 50  0000 C CNN
F 2 "" H 9525 1850 50  0001 C CNN
F 3 "" H 9525 1850 50  0001 C CNN
	1    9525 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9975 1050 9525 1050
$Comp
L Device:R R721
U 1 1 5DB78864
P 9525 2700
F 0 "R721" H 9600 2650 50  0000 L CNN
F 1 "30k9" H 9600 2750 50  0000 L CNN
F 2 "VehicleAccessoryController:R_0603" V 9455 2700 50  0001 C CNN
F 3 "~" H 9525 2700 50  0001 C CNN
	1    9525 2700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0126
U 1 1 5DB78A2C
P 9525 3350
F 0 "#PWR0126" H 9525 3100 50  0001 C CNN
F 1 "GND" H 9530 3177 50  0000 C CNN
F 2 "" H 9525 3350 50  0001 C CNN
F 3 "" H 9525 3350 50  0001 C CNN
	1    9525 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	9975 2550 9525 2550
Text Label 1000 4050 2    50   ~ 0
ACC_IN
$Comp
L power:GNDA #PWR0129
U 1 1 5DB94C72
P 1450 4325
F 0 "#PWR0129" H 1450 4075 50  0001 C CNN
F 1 "GNDA" H 1455 4152 50  0000 C CNN
F 2 "" H 1450 4325 50  0001 C CNN
F 3 "" H 1450 4325 50  0001 C CNN
	1    1450 4325
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0130
U 1 1 5DB94CF1
P 2550 3875
F 0 "#PWR0130" H 2550 3725 50  0001 C CNN
F 1 "+5V" H 2565 4048 50  0000 C CNN
F 2 "" H 2550 3875 50  0001 C CNN
F 3 "" H 2550 3875 50  0001 C CNN
	1    2550 3875
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0131
U 1 1 5DB94D36
P 2550 4325
F 0 "#PWR0131" H 2550 4075 50  0001 C CNN
F 1 "GND" H 2555 4152 50  0000 C CNN
F 2 "" H 2550 4325 50  0001 C CNN
F 3 "" H 2550 4325 50  0001 C CNN
	1    2550 4325
	1    0    0    -1  
$EndComp
$Comp
L VehicleAccessoryController:SI8710CC-B-IS U104
U 1 1 5DB926C2
P 2000 4100
F 0 "U104" H 1750 4400 50  0000 C CNN
F 1 "SI8710CC-B-IS" H 2000 3850 50  0000 C CNN
F 2 "VehicleAccessoryController:SOIC-8_NB" H 2000 4100 50  0001 C CNN
F 3 "" H 2000 4100 50  0001 C CNN
	1    2000 4100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R716
U 1 1 5DBA16F4
P 9075 1200
F 0 "R716" H 9150 1150 50  0000 L CNN
F 1 "22k1" H 9150 1250 50  0000 L CNN
F 2 "VehicleAccessoryController:R_0603" V 9005 1200 50  0001 C CNN
F 3 "~" H 9075 1200 50  0001 C CNN
	1    9075 1200
	-1   0    0    1   
$EndComp
$Comp
L Device:R R717
U 1 1 5DBA1A00
P 9075 2700
F 0 "R717" H 9150 2650 50  0000 L CNN
F 1 "22k1" H 9150 2750 50  0000 L CNN
F 2 "VehicleAccessoryController:R_0603" V 9005 2700 50  0001 C CNN
F 3 "~" H 9075 2700 50  0001 C CNN
	1    9075 2700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0132
U 1 1 5DBA7C54
P 9075 1850
F 0 "#PWR0132" H 9075 1600 50  0001 C CNN
F 1 "GND" H 9080 1677 50  0000 C CNN
F 2 "" H 9075 1850 50  0001 C CNN
F 3 "" H 9075 1850 50  0001 C CNN
	1    9075 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0133
U 1 1 5DBA7D6F
P 9075 3350
F 0 "#PWR0133" H 9075 3100 50  0001 C CNN
F 1 "GND" H 9080 3177 50  0000 C CNN
F 2 "" H 9075 3350 50  0001 C CNN
F 3 "" H 9075 3350 50  0001 C CNN
	1    9075 3350
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:BSS138 Q201
U 1 1 5DBA851A
P 8975 1575
F 0 "Q201" H 9180 1621 50  0000 L CNN
F 1 "BSS138" H 9180 1530 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 9175 1500 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/BS/BSS138.pdf" H 8975 1575 50  0001 L CNN
	1    8975 1575
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:BSS138 Q202
U 1 1 5DBB6C5A
P 8975 3075
F 0 "Q202" H 9180 3121 50  0000 L CNN
F 1 "BSS138" H 9180 3030 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 9175 3000 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/BS/BSS138.pdf" H 8975 3075 50  0001 L CNN
	1    8975 3075
	1    0    0    -1  
$EndComp
Wire Wire Line
	9075 1350 9075 1375
Wire Wire Line
	9075 2850 9075 2875
Wire Wire Line
	9525 1050 9075 1050
Connection ~ 9525 1050
Wire Wire Line
	9525 2550 9075 2550
Connection ~ 9525 2550
$Comp
L Device:R R708
U 1 1 5DBF1B2F
P 8475 1575
F 0 "R708" V 8625 1475 50  0000 L CNN
F 1 "1k" V 8550 1525 50  0000 L CNN
F 2 "VehicleAccessoryController:R_0603" V 8405 1575 50  0001 C CNN
F 3 "~" H 8475 1575 50  0001 C CNN
	1    8475 1575
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R709
U 1 1 5DBF2553
P 8475 3075
F 0 "R709" V 8625 2975 50  0000 L CNN
F 1 "1k" V 8550 3025 50  0000 L CNN
F 2 "VehicleAccessoryController:R_0603" V 8405 3075 50  0001 C CNN
F 3 "~" H 8475 3075 50  0001 C CNN
	1    8475 3075
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8625 3075 8700 3075
Wire Wire Line
	8175 3075 8325 3075
Wire Wire Line
	8625 1575 8700 1575
Wire Wire Line
	8175 1575 8325 1575
$Comp
L Device:R R712
U 1 1 5DC2F355
P 8475 1850
F 0 "R712" V 8625 1750 50  0000 L CNN
F 1 "1M" V 8550 1800 50  0000 L CNN
F 2 "VehicleAccessoryController:R_0603" V 8405 1850 50  0001 C CNN
F 3 "~" H 8475 1850 50  0001 C CNN
	1    8475 1850
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R713
U 1 1 5DC2F52F
P 8475 3350
F 0 "R713" V 8625 3250 50  0000 L CNN
F 1 "1M" V 8550 3300 50  0000 L CNN
F 2 "VehicleAccessoryController:R_0603" V 8405 3350 50  0001 C CNN
F 3 "~" H 8475 3350 50  0001 C CNN
	1    8475 3350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8700 3075 8700 3350
Wire Wire Line
	8700 3350 8625 3350
Connection ~ 8700 3075
Wire Wire Line
	8700 3075 8775 3075
Wire Wire Line
	8700 1575 8700 1850
Wire Wire Line
	8700 1850 8625 1850
Connection ~ 8700 1575
Wire Wire Line
	8700 1575 8775 1575
$Comp
L power:GND #PWR0138
U 1 1 5DC51AE7
P 8175 3350
F 0 "#PWR0138" H 8175 3100 50  0001 C CNN
F 1 "GND" H 8180 3177 50  0000 C CNN
F 2 "" H 8175 3350 50  0001 C CNN
F 3 "" H 8175 3350 50  0001 C CNN
	1    8175 3350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0139
U 1 1 5DC5214A
P 8175 1850
F 0 "#PWR0139" H 8175 1600 50  0001 C CNN
F 1 "GND" H 8180 1677 50  0000 C CNN
F 2 "" H 8175 1850 50  0001 C CNN
F 3 "" H 8175 1850 50  0001 C CNN
	1    8175 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9825 1150 9825 1850
Wire Wire Line
	9525 1350 9525 1850
Wire Wire Line
	9075 1775 9075 1850
Wire Wire Line
	8175 1850 8325 1850
Wire Wire Line
	9825 2650 9825 3350
Wire Wire Line
	9525 2850 9525 3350
Wire Wire Line
	9075 3275 9075 3350
Wire Wire Line
	8175 3350 8325 3350
Wire Wire Line
	1450 4050 1600 4050
Wire Wire Line
	1600 4150 1450 4150
Wire Wire Line
	1450 4150 1450 4325
Wire Wire Line
	2400 4250 2550 4250
Wire Wire Line
	2550 4250 2550 4325
Wire Wire Line
	2400 3950 2550 3950
Wire Wire Line
	2550 3950 2550 3875
Text Label 2850 2725 2    50   ~ 0
CTRL
$Comp
L power:GNDA #PWR0142
U 1 1 5DD5360A
P 3425 2900
F 0 "#PWR0142" H 3425 2650 50  0001 C CNN
F 1 "GNDA" H 3430 2727 50  0000 C CNN
F 2 "" H 3425 2900 50  0001 C CNN
F 3 "" H 3425 2900 50  0001 C CNN
	1    3425 2900
	1    0    0    -1  
$EndComp
Text Label 3425 2625 0    50   ~ 0
SHUTDOWN
Wire Wire Line
	2850 2725 2950 2725
Wire Wire Line
	3425 2625 3350 2625
Wire Wire Line
	3425 2900 3425 2825
Wire Wire Line
	3425 2825 3350 2825
Text Label 8175 3075 2    50   ~ 0
PARK_REAR
$Comp
L Device:R R726
U 1 1 5DCC21A2
P 1775 5225
F 0 "R726" H 1850 5250 50  0000 L CNN
F 1 "1k" H 1900 5175 50  0000 L CNN
F 2 "VehicleAccessoryController:R_0603" V 1705 5225 50  0001 C CNN
F 3 "~" H 1775 5225 50  0001 C CNN
	1    1775 5225
	1    0    0    -1  
$EndComp
$Comp
L Device:R R724
U 1 1 5DCD1559
P 1300 4050
F 0 "R724" V 1450 3950 50  0000 L CNN
F 1 "1k" V 1375 4000 50  0000 L CNN
F 2 "VehicleAccessoryController:R_0603" V 1230 4050 50  0001 C CNN
F 3 "~" H 1300 4050 50  0001 C CNN
	1    1300 4050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1000 4050 1150 4050
$Comp
L VehicleAccessoryController:CP2102N-A02-GQFN24 U105
U 1 1 5DBBE1B8
P 8725 5400
F 0 "U105" H 8325 6200 50  0000 C CNN
F 1 "CP2102N-A02-GQFN24" H 8725 4825 50  0000 C CNN
F 2 "Package_DFN_QFN:QFN-24-1EP_4x4mm_P0.5mm_EP2.6x2.6mm" H 7475 4750 50  0001 L CNN
F 3 "http://www.silabs.com/support%20documents/technicaldocs/cp2102n-datasheet.pdf" H 8775 4350 50  0001 C CNN
	1    8725 5400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C805
U 1 1 5DC3FCAA
P 2225 5225
F 0 "C805" H 2350 5275 50  0000 L CNN
F 1 "0.1uF" H 2350 5200 50  0000 L CNN
F 2 "VehicleAccessoryController:C_0603" H 2263 5075 50  0001 C CNN
F 3 "~" H 2225 5225 50  0001 C CNN
	1    2225 5225
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0144
U 1 1 5DC3FCB0
P 2225 5000
F 0 "#PWR0144" H 2225 4850 50  0001 C CNN
F 1 "+5V" H 2240 5173 50  0000 C CNN
F 2 "" H 2225 5000 50  0001 C CNN
F 3 "" H 2225 5000 50  0001 C CNN
	1    2225 5000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0145
U 1 1 5DC3FCB6
P 2225 5450
F 0 "#PWR0145" H 2225 5200 50  0001 C CNN
F 1 "GND" H 2230 5277 50  0000 C CNN
F 2 "" H 2225 5450 50  0001 C CNN
F 3 "" H 2225 5450 50  0001 C CNN
	1    2225 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2225 5000 2225 5075
Wire Wire Line
	2225 5375 2225 5450
$Comp
L power:+5V #PWR0146
U 1 1 5DC61045
P 1775 5000
F 0 "#PWR0146" H 1775 4850 50  0001 C CNN
F 1 "+5V" H 1790 5173 50  0000 C CNN
F 2 "" H 1775 5000 50  0001 C CNN
F 3 "" H 1775 5000 50  0001 C CNN
	1    1775 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1775 5075 1775 5000
Text Label 2775 3750 0    50   ~ 0
ACC
Text Label 1775 5625 2    50   ~ 0
ACC
$Comp
L Device:R R725
U 1 1 5DCA412C
P 2775 4000
F 0 "R725" H 2850 4025 50  0000 L CNN
F 1 "160R" H 2850 3950 50  0000 L CNN
F 2 "VehicleAccessoryController:R_0603" V 2705 4000 50  0001 C CNN
F 3 "~" H 2775 4000 50  0001 C CNN
	1    2775 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 4150 2775 4150
Wire Wire Line
	2775 3850 2775 3750
Wire Wire Line
	1775 5375 1775 5625
Text Label 9425 5050 0    50   ~ 0
CP2102N_RXD
Text Label 9425 5150 0    50   ~ 0
CP2102N_TXD
Wire Wire Line
	8175 5750 8025 5750
Wire Wire Line
	8025 5750 8025 5850
Wire Wire Line
	8175 5850 8025 5850
Connection ~ 8025 5850
Wire Wire Line
	8025 5850 8025 5925
Wire Wire Line
	8175 5450 7500 5450
Wire Wire Line
	7500 5550 8175 5550
$Comp
L Device:C C807
U 1 1 5DC49B12
P 4900 7050
F 0 "C807" H 4575 7125 50  0000 L CNN
F 1 "0.1uF" H 4575 7025 50  0000 L CNN
F 2 "VehicleAccessoryController:C_0603" H 4938 6900 50  0001 C CNN
F 3 "~" H 4900 7050 50  0001 C CNN
	1    4900 7050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C806
U 1 1 5DC49D8C
P 4450 7050
F 0 "C806" H 4125 7125 50  0000 L CNN
F 1 "4.7uF" H 4125 7025 50  0000 L CNN
F 2 "VehicleAccessoryController:C_0603" H 4488 6900 50  0001 C CNN
F 3 "~" H 4450 7050 50  0001 C CNN
	1    4450 7050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R728
U 1 1 5DC4A37C
P 9425 4600
F 0 "R728" H 9500 4625 50  0000 L CNN
F 1 "1k" H 9550 4550 50  0000 L CNN
F 2 "VehicleAccessoryController:R_0603" V 9355 4600 50  0001 C CNN
F 3 "~" H 9425 4600 50  0001 C CNN
	1    9425 4600
	1    0    0    -1  
$EndComp
$Comp
L power:VBUS #PWR0148
U 1 1 5DC4AD62
P 8025 4675
F 0 "#PWR0148" H 8025 4525 50  0001 C CNN
F 1 "VBUS" H 8040 4848 50  0000 C CNN
F 2 "" H 8025 4675 50  0001 C CNN
F 3 "" H 8025 4675 50  0001 C CNN
	1    8025 4675
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 6825 4450 6900
Wire Wire Line
	4900 6825 4900 6900
Wire Wire Line
	4450 7200 4450 7275
Wire Wire Line
	4900 7200 4900 7275
Wire Wire Line
	8025 4675 8025 4750
Wire Wire Line
	8025 4750 8175 4750
$Comp
L power:VDD #PWR0151
U 1 1 5DC8C969
P 7650 4675
F 0 "#PWR0151" H 7650 4525 50  0001 C CNN
F 1 "VDD" H 7667 4848 50  0000 C CNN
F 2 "" H 7650 4675 50  0001 C CNN
F 3 "" H 7650 4675 50  0001 C CNN
	1    7650 4675
	1    0    0    -1  
$EndComp
$Comp
L Device:C C809
U 1 1 5DCBD7BA
P 5850 7050
F 0 "C809" H 5525 7125 50  0000 L CNN
F 1 "0.1uF" H 5525 7025 50  0000 L CNN
F 2 "VehicleAccessoryController:C_0603" H 5888 6900 50  0001 C CNN
F 3 "~" H 5850 7050 50  0001 C CNN
	1    5850 7050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C808
U 1 1 5DCBD7C0
P 5400 7050
F 0 "C808" H 5075 7125 50  0000 L CNN
F 1 "4.7uF" H 5075 7025 50  0000 L CNN
F 2 "VehicleAccessoryController:C_0603" H 5438 6900 50  0001 C CNN
F 3 "~" H 5400 7050 50  0001 C CNN
	1    5400 7050
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0152
U 1 1 5DCBD7C6
P 5400 6825
F 0 "#PWR0152" H 5400 6675 50  0001 C CNN
F 1 "VDD" H 5417 6998 50  0000 C CNN
F 2 "" H 5400 6825 50  0001 C CNN
F 3 "" H 5400 6825 50  0001 C CNN
	1    5400 6825
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0153
U 1 1 5DCBD7CC
P 5850 6825
F 0 "#PWR0153" H 5850 6675 50  0001 C CNN
F 1 "VDD" H 5867 6998 50  0000 C CNN
F 2 "" H 5850 6825 50  0001 C CNN
F 3 "" H 5850 6825 50  0001 C CNN
	1    5850 6825
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 6825 5400 6900
Wire Wire Line
	5850 6825 5850 6900
Wire Wire Line
	5400 7200 5400 7275
Wire Wire Line
	5850 7200 5850 7275
$Comp
L power:VBUS #PWR0156
U 1 1 5DCC7AA9
P 4450 6825
F 0 "#PWR0156" H 4450 6675 50  0001 C CNN
F 1 "VBUS" H 4465 6998 50  0000 C CNN
F 2 "" H 4450 6825 50  0001 C CNN
F 3 "" H 4450 6825 50  0001 C CNN
	1    4450 6825
	1    0    0    -1  
$EndComp
$Comp
L power:VBUS #PWR0157
U 1 1 5DCC7B2C
P 4900 6825
F 0 "#PWR0157" H 4900 6675 50  0001 C CNN
F 1 "VBUS" H 4915 6998 50  0000 C CNN
F 2 "" H 4900 6825 50  0001 C CNN
F 3 "" H 4900 6825 50  0001 C CNN
	1    4900 6825
	1    0    0    -1  
$EndComp
$Comp
L Device:R R727
U 1 1 5DD23693
P 4850 5275
F 0 "R727" H 4925 5225 50  0000 L CNN
F 1 "1M" H 4975 5325 50  0000 L CNN
F 2 "VehicleAccessoryController:R_0603" V 4780 5275 50  0001 C CNN
F 3 "~" H 4850 5275 50  0001 C CNN
	1    4850 5275
	-1   0    0    1   
$EndComp
$Comp
L Device:C C810
U 1 1 5DD23699
P 5275 5275
F 0 "C810" H 4975 5325 50  0000 L CNN
F 1 "0.1nF" H 4975 5250 50  0000 L CNN
F 2 "VehicleAccessoryController:C_0603" H 5313 5125 50  0001 C CNN
F 3 "~" H 5275 5275 50  0001 C CNN
	1    5275 5275
	1    0    0    -1  
$EndComp
Wire Wire Line
	5275 5050 5275 5125
Wire Wire Line
	5275 5425 5275 5500
Wire Wire Line
	4850 5125 4850 5050
Wire Wire Line
	4850 5425 4850 5500
Wire Wire Line
	7100 5925 7100 5850
Wire Wire Line
	7200 5925 7200 5850
Text Label 7100 5925 2    50   ~ 0
SHIELD
Text Label 4850 5050 2    50   ~ 0
SHIELD
Text Label 5275 5050 2    50   ~ 0
SHIELD
$Comp
L power:VDD #PWR0161
U 1 1 5DD98C5E
P 9425 4375
F 0 "#PWR0161" H 9425 4225 50  0001 C CNN
F 1 "VDD" H 9442 4548 50  0000 C CNN
F 2 "" H 9425 4375 50  0001 C CNN
F 3 "" H 9425 4375 50  0001 C CNN
	1    9425 4375
	1    0    0    -1  
$EndComp
Wire Wire Line
	9275 4750 9425 4750
Wire Wire Line
	8175 4850 8025 4850
$Comp
L Power_Protection:SP0503BAHT D605
U 1 1 5DDF10C9
P 6200 5675
F 0 "D605" V 5975 5575 50  0000 L CNN
F 1 "SP0503BAHT" V 6425 5425 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-143" H 6425 5625 50  0001 L CNN
F 3 "http://www.littelfuse.com/~/media/files/littelfuse/technical%20resources/documents/data%20sheets/sp05xxba.pdf" H 6325 5800 50  0001 C CNN
	1    6200 5675
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 5400 6100 5475
Wire Wire Line
	6200 5400 6200 5475
Wire Wire Line
	6300 5400 6300 5475
Wire Wire Line
	6200 5875 6200 5950
Text Label 7725 5550 0    50   ~ 0
D-
Text Label 7725 5450 0    50   ~ 0
D+
Text Label 6300 5400 1    50   ~ 0
D-
Text Label 6200 5400 1    50   ~ 0
D+
$Comp
L power:VBUS #PWR0163
U 1 1 5DE2062B
P 6100 5400
F 0 "#PWR0163" H 6100 5250 50  0001 C CNN
F 1 "VBUS" H 6115 5573 50  0000 C CNN
F 2 "" H 6100 5400 50  0001 C CNN
F 3 "" H 6100 5400 50  0001 C CNN
	1    6100 5400
	1    0    0    -1  
$EndComp
NoConn ~ 8175 5250
NoConn ~ 8175 5350
NoConn ~ 9275 5250
NoConn ~ 9275 5450
NoConn ~ 8175 5150
NoConn ~ 9275 4850
NoConn ~ 9275 4950
$Comp
L Device:LED_ALT D603
U 1 1 5DCD21E8
P 3525 6900
F 0 "D603" H 3525 7025 50  0000 C CNN
F 1 "LED_ALT" H 3525 6750 50  0000 C CNN
F 2 "VehicleAccessoryController:LED_0603" H 3525 6900 50  0001 C CNN
F 3 "~" H 3525 6900 50  0001 C CNN
	1    3525 6900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R729
U 1 1 5DCD21FC
P 3075 6900
F 0 "R729" V 3225 6800 50  0000 L CNN
F 1 "90R" V 3150 6825 50  0000 L CNN
F 2 "VehicleAccessoryController:R_0603" V 3005 6900 50  0001 C CNN
F 3 "~" H 3075 6900 50  0001 C CNN
	1    3075 6900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3225 6900 3375 6900
Wire Wire Line
	3675 6900 3825 6900
Wire Wire Line
	3825 6900 3825 6825
Wire Wire Line
	2775 6900 2925 6900
$Comp
L Device:LED_ALT D604
U 1 1 5DCD220B
P 3525 7450
F 0 "D604" H 3525 7575 50  0000 C CNN
F 1 "LED_ALT" H 3525 7300 50  0000 C CNN
F 2 "VehicleAccessoryController:LED_0603" H 3525 7450 50  0001 C CNN
F 3 "~" H 3525 7450 50  0001 C CNN
	1    3525 7450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R730
U 1 1 5DCD221F
P 3075 7450
F 0 "R730" V 3225 7350 50  0000 L CNN
F 1 "90R" V 3150 7375 50  0000 L CNN
F 2 "VehicleAccessoryController:R_0603" V 3005 7450 50  0001 C CNN
F 3 "~" H 3075 7450 50  0001 C CNN
	1    3075 7450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3225 7450 3375 7450
Wire Wire Line
	3675 7450 3825 7450
Wire Wire Line
	3825 7450 3825 7375
Wire Wire Line
	2775 7450 2925 7450
$Comp
L power:VDD #PWR0164
U 1 1 5DD4F165
P 3825 6825
F 0 "#PWR0164" H 3825 6675 50  0001 C CNN
F 1 "VDD" H 3842 6998 50  0000 C CNN
F 2 "" H 3825 6825 50  0001 C CNN
F 3 "" H 3825 6825 50  0001 C CNN
	1    3825 6825
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0165
U 1 1 5DD53CD1
P 3825 7375
F 0 "#PWR0165" H 3825 7225 50  0001 C CNN
F 1 "VDD" H 3842 7548 50  0000 C CNN
F 2 "" H 3825 7375 50  0001 C CNN
F 3 "" H 3825 7375 50  0001 C CNN
	1    3825 7375
	1    0    0    -1  
$EndComp
Wire Wire Line
	9425 5750 9275 5750
Wire Wire Line
	9425 5850 9275 5850
Text Label 9425 5850 0    50   ~ 0
TX_LED
Text Label 9425 5750 0    50   ~ 0
RX_LED
Text Label 2775 7450 2    50   ~ 0
TX_LED
Text Label 2775 6900 2    50   ~ 0
RX_LED
Wire Wire Line
	9425 5350 9275 5350
$Comp
L Device:C C811
U 1 1 5DDADB58
P 9575 5350
F 0 "C811" V 9725 5250 50  0000 L CNN
F 1 "0.1uF" V 9800 5250 50  0000 L CNN
F 2 "VehicleAccessoryController:C_0603" H 9613 5200 50  0001 C CNN
F 3 "~" H 9575 5350 50  0001 C CNN
	1    9575 5350
	0    1    1    0   
$EndComp
Wire Wire Line
	9800 5350 9725 5350
Text Label 9800 5350 0    50   ~ 0
328P_RESET
NoConn ~ 9275 5550
NoConn ~ 9275 5650
Wire Wire Line
	9425 4375 9425 4450
$Comp
L power:VBUS #PWR0166
U 1 1 5DF328EA
P 7500 5150
F 0 "#PWR0166" H 7500 5000 50  0001 C CNN
F 1 "VBUS" H 7515 5323 50  0000 C CNN
F 2 "" H 7500 5150 50  0001 C CNN
F 3 "" H 7500 5150 50  0001 C CNN
	1    7500 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7500 5250 7500 5150
NoConn ~ 7500 5650
$Comp
L Device:R R732
U 1 1 5DF513E5
P 3900 5450
F 0 "R732" H 3975 5400 50  0000 L CNN
F 1 "47.5k" H 3975 5500 50  0000 L CNN
F 2 "VehicleAccessoryController:R_0603" V 3830 5450 50  0001 C CNN
F 3 "~" H 3900 5450 50  0001 C CNN
	1    3900 5450
	-1   0    0    1   
$EndComp
Wire Wire Line
	3900 5600 3900 5675
$Comp
L Device:R R731
U 1 1 5DF6578F
P 3900 5050
F 0 "R731" H 3975 5000 50  0000 L CNN
F 1 "22.1k" H 3975 5100 50  0000 L CNN
F 2 "VehicleAccessoryController:R_0603" V 3830 5050 50  0001 C CNN
F 3 "~" H 3900 5050 50  0001 C CNN
	1    3900 5050
	-1   0    0    1   
$EndComp
$Comp
L power:VBUS #PWR0168
U 1 1 5DF669C8
P 3900 4825
F 0 "#PWR0168" H 3900 4675 50  0001 C CNN
F 1 "VBUS" H 3915 4998 50  0000 C CNN
F 2 "" H 3900 4825 50  0001 C CNN
F 3 "" H 3900 4825 50  0001 C CNN
	1    3900 4825
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 4825 3900 4900
Wire Wire Line
	3900 5300 3900 5250
Wire Wire Line
	3900 5250 4050 5250
Connection ~ 3900 5250
Wire Wire Line
	3900 5250 3900 5200
Text Label 4050 5250 0    50   ~ 0
VBUS_DIV
Text Label 8025 4850 2    50   ~ 0
VBUS_DIV
Wire Wire Line
	7650 4675 7650 4950
Wire Wire Line
	7650 5050 8175 5050
Wire Wire Line
	8175 4950 7650 4950
Connection ~ 7650 4950
Wire Wire Line
	7650 4950 7650 5050
Wire Wire Line
	9275 5150 9425 5150
$Comp
L VehicleAccessoryController:Si8621BB-B-IS U106
U 1 1 5DC8033B
P 5550 3675
F 0 "U106" H 5200 3975 50  0000 C CNN
F 1 "Si8621BB-B-IS" H 5550 3375 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 5550 3275 50  0001 C CIN
F 3 "https://www.silabs.com/documents/public/data-sheets/si861x-2x-datasheet.pdf" H 5550 4075 50  0001 C CNN
	1    5550 3675
	1    0    0    -1  
$EndComp
Wire Wire Line
	9275 5050 9425 5050
Wire Wire Line
	4900 3625 5050 3625
Wire Wire Line
	4900 3725 5050 3725
Wire Wire Line
	4900 3900 4900 3825
Wire Wire Line
	4900 3825 5050 3825
Wire Wire Line
	4900 3450 4900 3525
Wire Wire Line
	4900 3525 5050 3525
Wire Wire Line
	6200 3450 6200 3525
Wire Wire Line
	6200 3525 6050 3525
Wire Wire Line
	6200 3625 6050 3625
Wire Wire Line
	6200 3725 6050 3725
Wire Wire Line
	6200 3900 6200 3825
Wire Wire Line
	6200 3825 6050 3825
Text Label 4900 3625 2    50   ~ 0
CP2102N_TXD
Text Label 4900 3725 2    50   ~ 0
CP2102N_RXD
Text Label 6200 3625 0    50   ~ 0
328P_RXD
Text Label 6200 3725 0    50   ~ 0
328P_TXD
$Comp
L power:GND #PWR0147
U 1 1 5DD399F8
P 6200 3900
F 0 "#PWR0147" H 6200 3650 50  0001 C CNN
F 1 "GND" H 6205 3727 50  0000 C CNN
F 2 "" H 6200 3900 50  0001 C CNN
F 3 "" H 6200 3900 50  0001 C CNN
	1    6200 3900
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0158
U 1 1 5DD3D68A
P 6200 3450
F 0 "#PWR0158" H 6200 3300 50  0001 C CNN
F 1 "+5V" H 6215 3623 50  0000 C CNN
F 2 "" H 6200 3450 50  0001 C CNN
F 3 "" H 6200 3450 50  0001 C CNN
	1    6200 3450
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR0159
U 1 1 5DD484EB
P 8025 5925
F 0 "#PWR0159" H 8025 5675 50  0001 C CNN
F 1 "GNDD" H 8029 5770 50  0000 C CNN
F 2 "" H 8025 5925 50  0001 C CNN
F 3 "" H 8025 5925 50  0001 C CNN
	1    8025 5925
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR0160
U 1 1 5DD4F6D1
P 7200 5925
F 0 "#PWR0160" H 7200 5675 50  0001 C CNN
F 1 "GNDD" H 7204 5770 50  0000 C CNN
F 2 "" H 7200 5925 50  0001 C CNN
F 3 "" H 7200 5925 50  0001 C CNN
	1    7200 5925
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR0162
U 1 1 5DD5106C
P 6200 5950
F 0 "#PWR0162" H 6200 5700 50  0001 C CNN
F 1 "GNDD" H 6204 5795 50  0000 C CNN
F 2 "" H 6200 5950 50  0001 C CNN
F 3 "" H 6200 5950 50  0001 C CNN
	1    6200 5950
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR0167
U 1 1 5DD51776
P 3900 5675
F 0 "#PWR0167" H 3900 5425 50  0001 C CNN
F 1 "GNDD" H 3904 5520 50  0000 C CNN
F 2 "" H 3900 5675 50  0001 C CNN
F 3 "" H 3900 5675 50  0001 C CNN
	1    3900 5675
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR0169
U 1 1 5DD51E38
P 5275 5500
F 0 "#PWR0169" H 5275 5250 50  0001 C CNN
F 1 "GNDD" H 5279 5345 50  0000 C CNN
F 2 "" H 5275 5500 50  0001 C CNN
F 3 "" H 5275 5500 50  0001 C CNN
	1    5275 5500
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR0170
U 1 1 5DD52497
P 4850 5500
F 0 "#PWR0170" H 4850 5250 50  0001 C CNN
F 1 "GNDD" H 4854 5345 50  0000 C CNN
F 2 "" H 4850 5500 50  0001 C CNN
F 3 "" H 4850 5500 50  0001 C CNN
	1    4850 5500
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR0171
U 1 1 5DD53F12
P 4900 3900
F 0 "#PWR0171" H 4900 3650 50  0001 C CNN
F 1 "GNDD" H 4904 3745 50  0000 C CNN
F 2 "" H 4900 3900 50  0001 C CNN
F 3 "" H 4900 3900 50  0001 C CNN
	1    4900 3900
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0172
U 1 1 5DD548D3
P 4900 3450
F 0 "#PWR0172" H 4900 3300 50  0001 C CNN
F 1 "VDD" H 4917 3623 50  0000 C CNN
F 2 "" H 4900 3450 50  0001 C CNN
F 3 "" H 4900 3450 50  0001 C CNN
	1    4900 3450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C813
U 1 1 5DD89F06
P 6725 3675
F 0 "C813" H 6850 3725 50  0000 L CNN
F 1 "0.1uF" H 6850 3650 50  0000 L CNN
F 2 "VehicleAccessoryController:C_0603" H 6763 3525 50  0001 C CNN
F 3 "~" H 6725 3675 50  0001 C CNN
	1    6725 3675
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0173
U 1 1 5DD89F10
P 6725 3450
F 0 "#PWR0173" H 6725 3300 50  0001 C CNN
F 1 "+5V" H 6740 3623 50  0000 C CNN
F 2 "" H 6725 3450 50  0001 C CNN
F 3 "" H 6725 3450 50  0001 C CNN
	1    6725 3450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0174
U 1 1 5DD89F1A
P 6725 3900
F 0 "#PWR0174" H 6725 3650 50  0001 C CNN
F 1 "GND" H 6730 3727 50  0000 C CNN
F 2 "" H 6725 3900 50  0001 C CNN
F 3 "" H 6725 3900 50  0001 C CNN
	1    6725 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6725 3450 6725 3525
Wire Wire Line
	6725 3825 6725 3900
$Comp
L Device:C C812
U 1 1 5DDC0899
P 4225 3675
F 0 "C812" H 3900 3725 50  0000 L CNN
F 1 "0.1uF" H 3900 3625 50  0000 L CNN
F 2 "VehicleAccessoryController:C_0603" H 4263 3525 50  0001 C CNN
F 3 "~" H 4225 3675 50  0001 C CNN
	1    4225 3675
	1    0    0    -1  
$EndComp
Wire Wire Line
	4225 3450 4225 3525
Wire Wire Line
	4225 3825 4225 3900
$Comp
L power:GNDD #PWR0175
U 1 1 5DDF2919
P 4225 3900
F 0 "#PWR0175" H 4225 3650 50  0001 C CNN
F 1 "GNDD" H 4229 3745 50  0000 C CNN
F 2 "" H 4225 3900 50  0001 C CNN
F 3 "" H 4225 3900 50  0001 C CNN
	1    4225 3900
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0176
U 1 1 5DDF328F
P 4225 3450
F 0 "#PWR0176" H 4225 3300 50  0001 C CNN
F 1 "VDD" H 4242 3623 50  0000 C CNN
F 2 "" H 4225 3450 50  0001 C CNN
F 3 "" H 4225 3450 50  0001 C CNN
	1    4225 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2725 1075 2725 1150
$Comp
L VehicleAccessoryController:USB_Mini-B J506
U 1 1 5DC8030C
P 7200 5450
F 0 "J506" H 7200 5800 50  0000 C CNN
F 1 "USB_Mini-B" V 6950 5450 50  0000 C CNN
F 2 "VehicleAccessoryController:AFCI_10033526_USB-mini" H 7350 5400 50  0001 C CNN
F 3 "" H 7350 5400 50  0001 C CNN
	1    7200 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1275 1250 1800 1250
Wire Wire Line
	1800 1250 1800 1325
Text Label 1325 2825 2    50   ~ 0
PARK_FRONT
Text Label 1325 2925 2    50   ~ 0
PARK_REAR
Text Label 1325 3025 2    50   ~ 0
PARK_STATUS
Text Label 1325 1950 2    50   ~ 0
CTRL
$Comp
L power:+5V #PWR0101
U 1 1 5DD0F1D4
P 1325 2650
F 0 "#PWR0101" H 1325 2500 50  0001 C CNN
F 1 "+5V" H 1340 2823 50  0000 C CNN
F 2 "" H 1325 2650 50  0001 C CNN
F 3 "" H 1325 2650 50  0001 C CNN
	1    1325 2650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5DD0FA7E
P 1325 3500
F 0 "#PWR0103" H 1325 3250 50  0001 C CNN
F 1 "GND" H 1330 3327 50  0000 C CNN
F 2 "" H 1325 3500 50  0001 C CNN
F 3 "" H 1325 3500 50  0001 C CNN
	1    1325 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GNDA #PWR0104
U 1 1 5DD11228
P 1325 2125
F 0 "#PWR0104" H 1325 1875 50  0001 C CNN
F 1 "GNDA" H 1330 1952 50  0000 C CNN
F 2 "" H 1325 2125 50  0001 C CNN
F 3 "" H 1325 2125 50  0001 C CNN
	1    1325 2125
	1    0    0    -1  
$EndComp
Text Label 1325 3325 2    50   ~ 0
328P_RXD
Text Label 1325 3225 2    50   ~ 0
328P_TXD
$Comp
L Connector:Conn_01x02_Male J507
U 1 1 5DD26DED
P 1675 1950
F 0 "J507" H 1850 2075 50  0000 R CNN
F 1 "Conn_01x02_Male" H 1650 1900 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 1675 1950 50  0001 C CNN
F 3 "~" H 1675 1950 50  0001 C CNN
	1    1675 1950
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1325 1950 1475 1950
Wire Wire Line
	1325 2125 1325 2050
Wire Wire Line
	1325 2050 1475 2050
Wire Wire Line
	1325 2650 1325 2725
Wire Wire Line
	1325 2825 1475 2825
Wire Wire Line
	1325 2925 1475 2925
Wire Wire Line
	1325 3025 1475 3025
Wire Wire Line
	1325 3125 1475 3125
Wire Wire Line
	1325 3225 1475 3225
Wire Wire Line
	1325 3325 1475 3325
Wire Wire Line
	1325 3500 1325 3425
Wire Wire Line
	1325 3425 1475 3425
Text Label 1325 3125 2    50   ~ 0
ACC
$Comp
L Connector:Conn_01x08_Male J508
U 1 1 5DCEE74B
P 1675 3025
F 0 "J508" H 1800 3450 50  0000 C CNN
F 1 "Conn_01x08_Male" V 1625 2975 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical" H 1675 3025 50  0001 C CNN
F 3 "~" H 1675 3025 50  0001 C CNN
	1    1675 3025
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1475 2725 1325 2725
$Comp
L Connector:TestPoint TP1
U 1 1 5DD0862F
P 8950 6400
F 0 "TP1" V 8875 6475 50  0000 L CNN
F 1 "TestPoint" V 9025 6375 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.0x1.0mm" H 9150 6400 50  0001 C CNN
F 3 "~" H 9150 6400 50  0001 C CNN
	1    8950 6400
	0    1    1    0   
$EndComp
Text Label 8800 6400 2    50   ~ 0
328P_RESET
Wire Wire Line
	8800 6400 8950 6400
$Comp
L power:GNDD #PWR?
U 1 1 5DD412B8
P 4450 7275
F 0 "#PWR?" H 4450 7025 50  0001 C CNN
F 1 "GNDD" H 4454 7120 50  0000 C CNN
F 2 "" H 4450 7275 50  0001 C CNN
F 3 "" H 4450 7275 50  0001 C CNN
	1    4450 7275
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR?
U 1 1 5DD434DA
P 4900 7275
F 0 "#PWR?" H 4900 7025 50  0001 C CNN
F 1 "GNDD" H 4904 7120 50  0000 C CNN
F 2 "" H 4900 7275 50  0001 C CNN
F 3 "" H 4900 7275 50  0001 C CNN
	1    4900 7275
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR?
U 1 1 5DD437E8
P 5400 7275
F 0 "#PWR?" H 5400 7025 50  0001 C CNN
F 1 "GNDD" H 5404 7120 50  0000 C CNN
F 2 "" H 5400 7275 50  0001 C CNN
F 3 "" H 5400 7275 50  0001 C CNN
	1    5400 7275
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR?
U 1 1 5DD43A92
P 5850 7275
F 0 "#PWR?" H 5850 7025 50  0001 C CNN
F 1 "GNDD" H 5854 7120 50  0000 C CNN
F 2 "" H 5850 7275 50  0001 C CNN
F 3 "" H 5850 7275 50  0001 C CNN
	1    5850 7275
	1    0    0    -1  
$EndComp
$EndSCHEMATC
