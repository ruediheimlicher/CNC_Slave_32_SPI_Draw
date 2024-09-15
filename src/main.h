//
//  expo.h
//  DOG_LCD
//
//  Created by Ruedi Heimlicher on 22.11.2013.
//
//
#include <stdint.h>



typedef struct canal_struct 
{
  uint16_t lx;
  uint16_t ly;
  uint16_t rx;
  uint16_t ry;

  uint8_t digi;

  uint16_t x;
  uint16_t y;
} canal_struct;




// bresenham

volatile uint8_t bresenhamstatus = 0x00; // relevanter motor, in Abschnittladen:bres gesetzt

volatile uint16_t bresenham_errAB = 0; // error A,B
volatile uint16_t bresenham_e2AB = 0;  // check A,B

volatile uint16_t bresenham_errCD = 0;
volatile uint16_t bresenham_e2CD = 0;

volatile uint16_t StepStartA = 0; // startwert am Anfang des Abschnittes
volatile uint16_t StepStartC = 0;

// Seite A
volatile int16_t xA, yA, tA, dxA, dyA, incxA, incyA, pdxA, pdyA, ddxA, ddyA, deltaslowdirectionA, deltafastdirectionA, errA;

volatile uint16_t deltafastdelayA = 0; // aktueller delay
volatile uint16_t bres_delayA = 0;     // steps fuer fastdirection
volatile uint16_t bres_counterA = 0;   // zaehler fuer fastdirection

uint16_t stepdurA = 0;
uint16_t stepdurB = 0;
uint16_t stepdurC = 0;
uint16_t stepdurD = 0;

// Seite B
volatile int16_t xB, yB, tB, dxB, dyB, incxB, incyB, pdxB, pdyB, ddxB, ddyB, deltaslowdirectionB, deltafastdirectionB, errB;

volatile uint16_t deltafastdelayB = 0; // aktueller delay
volatile uint16_t bres_delayB = 0;     // steps fuer fastdirection
volatile uint16_t bres_counterB = 0;   // zaehler fuer fastdirection

volatile int16_t xC, yC, tC, dxC, dyC, incxC, incyC, pdxC, pdyC, ddxC, ddyC, deltaslowdirectionC, deltafastdirectionC, errC;
volatile int16_t xD, yD, tD, dxD, dyD, incxD, incyD, pdxD, pdyD, ddxD, ddyD, deltaslowdirectionD, deltafastdirectionD, errD;


volatile uint16_t deltafastdelayC = 0; // aktueller delay
volatile uint16_t bres_delayC = 0;     // steps fuer fastdirection
volatile uint16_t bres_counterC = 0;   // zaehler fuer fastdirection

volatile uint16_t deltafastdelayD = 0; // aktueller delay
volatile uint16_t bres_delayD = 0;     // steps fuer fastdirection
volatile uint16_t bres_counterD = 0;   // zaehler fuer fastdirection


volatile uint8_t timerstatus = 0;

volatile uint8_t status = 0;

volatile uint8_t pfeilstatus = 0;
volatile uint8_t tastaturstatus = 0;
volatile uint8_t tastaturindex = 0; // counter for impuls/pause


volatile uint8_t PWM = 0;
static volatile uint8_t pwmposition = 0;
static volatile uint8_t pwmdivider = 0;

// CNC

volatile uint16_t CounterA = 0; // Zaehler fuer Delay von Motor A
volatile uint16_t CounterB = 0; // Zaehler fuer Delay von Motor B
volatile uint16_t CounterC = 0; // Zaehler fuer Delay von Motor C
volatile uint16_t CounterD = 0; // Zaehler fuer Delay von Motor D

volatile uint32_t DelayA = 24; // Delay von Motor A
volatile uint32_t DelayB = 24; // Delay von Motor B
volatile uint32_t DelayC = 24; // Delay von Motor C
volatile uint32_t DelayD = 24; // Delay von Motor D

volatile uint32_t StepCounterA = 0; // Zaehler fuer Schritte von Motor A
volatile uint32_t StepCounterB = 0; // Zaehler fuer Schritte von Motor B
volatile uint32_t StepCounterC = 0; // Zaehler fuer Schritte von Motor C
volatile uint32_t StepCounterD = 0; // Zaehler fuer Schritte von Motor D

volatile uint8_t richtung = 0;

volatile uint8_t richtungA = 0; // Richtung Motor A
volatile uint8_t richtungB = 0; // Richtung Motor B
volatile uint8_t homestatus = 0;

volatile uint8_t parallelcounter = 0;
volatile uint8_t parallelstatus = 0; // Status des Thread

volatile uint16_t timerintervall = TIMERINTERVALL;
volatile uint16_t timerintervall_SLOW = 0; // Intervall klein
volatile uint16_t timerintervall_FAST = 0; // Intervall gross

// Ramp

volatile uint16_t ramptimerintervall = TIMERINTERVALL;

volatile uint8_t rampstatus = 0;
// volatile uint8_t           RampZeit = RAMPZEIT;
// volatile uint8_t           RampFaktor = RAMPFAKTOR;
volatile uint32_t rampstepstart = 0; // Stepcounter am Anfang
// volatile uint32_t          ramptimercounter=0;  // laufender counter  fuer Rampanpassung
// volatile uint32_t          //ramptimerdelay = 100;  // Takt fuer Rampanpassung
uint8_t rampschritt = 2;
volatile uint16_t rampbreite = 10; // anzahl Schritte der Ramp. Wird beim Start bestimmt und fuer das Ende verwendet

volatile uint32_t rampendstep = 0; // Beginn der Endramp. Wird in Abschnittladen bestimmt

uint8_t richtungstatus = 0;
uint8_t oldrichtungstatus = 0;
