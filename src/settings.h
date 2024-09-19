//
//  settings.h
//  Stepper32
//
//  Created by Ruedi Heimlicher on 07.05.2020.
//  Copyright © 2020 Ruedi Heimlicher. All rights reserved.
//

#ifndef settings_h
#define settings_h

#define LOOPLED 14
#define TIMER0_STARTWERT   0x40

#define JOYSTICK 0

#define SERVO 1
#define OLED 0


#define EEPROMCALIB 0x10

// Stepper A


 // neu 200730
 #define MC_STEP        6
 #define MC_RI          7
 #define MC_EN          8
 
 //Pins 

 #define MA_STEP        0
 #define MA_RI          1
 #define MA_EN          2
 
// Stepper B
 #define MB_STEP         3
 #define MB_RI           4
 #define MB_EN           5
 
 #define END_A0_PIN      20           // Bit fuer Startanschlag bei Motor A
 #define END_B0_PIN      21           // Bit fuer Endanschlag bei A1
 
#define DC_PWM            9 // war 22

// 10,11,12: SPI
// #define MD_STEP            9           // PIN auf Stepperport 2
// 18, 19: I2C
//#define MD_RI              10
#define MD_EN              30

#define END_A1_PIN         16          // Anschlagstatus:  Bit fuer Endanschlag bei Motor A
#define END_B1_PIN         17         // Anschlagstatus:  Bit fuer Endanschlag bei D0

#define SERVO_PIN           6


//#define TASTE0            0   // HALT-Bit Motor A
//#define TASTE1            1


#define STEPDUR 250

#define STARTDELAYBIT       0
#define HICOUNTBIT          1

#define LOAD_NEXT          5  // Bit fuer Laden des naechsten Abschnitts in der loop
#define LOAD_LAST          6  // Bit fuer Laden des letzten Abschnitts in der loop

// auf Stepperport 1
#define END_A0          4       //  Bit fuer Endanschlag A0 
#define END_B0          5       //           Endanschlag B0 


// Bits fuer endanschlagstatus in AnschlagVonEndPin
#define ANSCHLAG_A0     0
#define ANSCHLAG_A1     1
#define ANSCHLAG_B0     2
#define ANSCHLAG_B1     3



// Auf Stepperport 2


#define END_C0          6       //  Bit fuer Endanschlag C0 
#define END_D0          7       //           Endanschlag D0 

#define RIGHT       1
#define UP          2
#define LEFT        3
#define DOWN        4

#define RICHTUNG_A   0 // Motor A pos
#define RICHTUNG_B   1 // Motor B pos
#define RICHTUNG_C   2 // 
#define RICHTUNG_D   3

#define MOTOR_A     0
#define MOTOR_B     1
#define MOTOR_C     2
#define MOTOR_D     3


#define HALT_PIN           0

#define COUNT_A            0 // 4      // Motorstatus:   Schritte von Motor A zaehlen
#define COUNT_B            1 // 5      // Motorstatus:   Schritte von Motor B zaehlen

#define COUNT_C            2 // 4      // Motorstatus:   Schritte von Motor C zaehlen
#define COUNT_D            3 // 5      // Motorstatus:   Schritte von Motor D zaehlen

#define COUNT_END          6
#define COUNT_LAST         7


#define TIMER_ON           1 // Bit fuer timerfunktion start

#define DC                  7    // DC ON: LO
#define STROM               4    // Stepperstrom ON: HI

#define GO_HOME            3     // Bit fuer befehl beginn home auf cncstatus
#define DC_DIVIDER         1      // teilt die pwm-Frequenz in ISR




// Ringbuffer
#define RINGBUFFERTIEFE    4
#define READYBIT           0        // buffer kann Daten aufnehmen
#define FULLBIT            1        // Buffer ist voll
#define STARTBIT           2        // Buffer ist geladen
#define RINGBUFFERBIT      3        // Ringbuffer wird verwendet
#define LASTBIT            4        // Letzter Abschnitt  ist geladen
#define ENDBIT             5        // Letzter Abschnitt  ist abgearbeitet
#define STOPBIT            6        // Ablauf stoppen
#define FIRSTBIT           7


#define OSZI_PULS_A        11
#define OSZI_PULS_B        11
#define OSZI_PULS_C        11
#define OSZI_PULS_D        11


#define THREAD_COUNT_BIT   0

#define TIMERINTERVALL 196

// Ramp
#define RAMP_OK      1 // Ramp einschalten
#define RAMPFAKTOR   2 // Verlaengerung der delayzeit
#define RAMPZEIT     800 // Mindestdauer fuer Ramp

#define RAMPSTARTBIT 1
#define RAMPENDBIT 2
#define RAMPEND0BIT 3 // Beginn der Endrampe
#define RAMPOKBIT    7
#define RAMPSCHRITT  1


#define DEVICE_MILL  1
#define DEVICE_JOY  2

#define VORZEICHEN_X   0
#define VORZEICHEN_Y   1


// Zeichnen
#define ACHSE0_BYTE_H   0
#define ACHSE0_BYTE_L   1
#define ACHSE0_START_BYTE_H   2
#define ACHSE0_START_BYTE_L   3

 
#define ACHSE1_BYTE_H   4
#define ACHSE1_BYTE_L   5
#define ACHSE1_START_BYTE_H   6
#define ACHSE1_START_BYTE_L   7

#define ACHSE2_BYTE_H   8
#define ACHSE2_BYTE_L   9
#define ACHSE2_START_BYTE_H   10
#define ACHSE2_START_BYTE_L   11

#define  ACHSE3_BYTE_H  12
#define  ACHSE3_BYTE_L  13

#define  INDEX_BYTE_H  18
#define  INDEX_BYTE_L  19

/*
// 9er-Keyboard
#define TASTE1     67
#define TASTE2     109
#define TASTE3     163
#define TASTE4     253
#define TASTE5     360
#define TASTE6     484
#define TASTE7     628
#define TASTE8     742
#define TASTE9     827
#define TASTEL     899
#define TASTE0     946
#define TASTER     993
*/

// 12er-Keyboard

#define TASTEX  10

// Joystick
/*
#define JOYSTICKTASTE1 25
#define JOYSTICKTASTE2 43
#define JOYSTICKTASTE3 69
#define JOYSTICKTASTE4 89
#define JOYSTICKTASTE5 112
#define JOYSTICKTASTE6 135
#define JOYSTICKTASTE7 157
#define JOYSTICKTASTE8 187
#define JOYSTICKTASTE9 211
*/
#define JOYSTICKTASTE1  17
#define JOYSTICKTASTE2  27
#define JOYSTICKTASTE3  41
#define JOYSTICKTASTE4  63
#define JOYSTICKTASTE5  90
#define JOYSTICKTASTE6  120
#define JOYSTICKTASTE7  155
#define JOYSTICKTASTE8  185
#define JOYSTICKTASTE9  206
#define JOYSTICKTASTE10  224
#define JOYSTICKTASTE11  236
#define JOYSTICKTASTE12  248
#define JOYSTICKTASTEL  250
#define JOYSTICKTASTER  250
#define JOYSTICKTASTE0  250


#define OLED_RESET   -1
#define OLED_CS      14
#define OLED_DC      12
#define OLED_CLK     13
#define OLED_MOSI    11


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



#define TASTEOK            1
#define AKTIONOK           2
#define UPDATEOK           3



/*
#define TASTE1  56
#define TASTE2  78
#define TASTE3  101
#define TASTE4  124
#define TASTE5  154
#define TASTE6  182
#define TASTE7  203
#define TASTE8  220
#define TASTE9  240

#define TASTEL     250
#define TASTE0     250
#define TASTER     250
*/

// Mill
/*
#define TASTE1  17
#define TASTE2  27
#define TASTE3  41
#define TASTE4  63
#define TASTE5  90
#define TASTE6  120
#define TASTE7  155
#define TASTE8  185
#define TASTE9  206
#define TASTEL  224
#define TASTE0  236
#define TASTER  248
*/

#define KEY1  18
#define KEY2  27
#define KEY3  41
#define KEY4  63
#define KEY5  90
#define KEY6  120
#define KEY7  155
#define KEY8  185
#define KEY9  206
#define KEYL  224
#define KEY0  236
#define KEYR  248



// von Mill32

//#define OSZI_PULS_A        9


#define TASTENSTARTIMPULSDAUER   3000 // Beginn Rampe
#define ENDIMPULSDAUER     20
#define TASTENENDIMPULSDAUER     800

#define IMPULSBREITE  150

# define RAMPDELAY 10 // delay fuer Reduktion Impulsdauer


// display
#define TASTE_Y  40
#define TASTE_X  60
#define CNC_Y    60

#define ANSCHLAG_X 70 
#define ANSCHLAG_Y 60 

#define JOYSTICK_Y  60
#define CALIB_Y  65
#define CALIB_X  0
#define CALIB_H   40
#define CALIB_W   115

#define SERVO_CODE    6
#define SERVO_UP      0
#define SERVO_DOWN    1
#define SERVO_SCHRITT 1
#define SERVO_DIV     0x0C

#define SERVO_MIN   100
#define SERVO_MAX   200
/*
#define  HYP_BYTE_H  22 // Hypotenuse
#define  HYP_BYTE_L 23


#define  STEPS_BYTE_H  26
#define  STEPS_BYTE_L  27
*/



#define JOYSTICKSTARTIMPULS   200 // impuls bei Start Joystick von Tastatur aus
#define JOYSTICKIMPULS        200 // impulsdauer
#define JOYSTICKTOTBEREICH    25
//#define JOYSTICKMINIMPULS     1000
#define JOYSTICKMAXDIFF       4800
#define JOYSTICKMAXTICKS      6000


#endif /* settings_h */
