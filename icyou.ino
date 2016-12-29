/*
  This sketch programs the icbob robot to move forward until the sonar
  detects an obstacle. Then it does a 90 deg turn and continues forward.

  It pulses an LED while walking, shines it while turning and plays a little march whenever
  it resumes walking forward.

  Written by The Bridgeville Delaware Public Library Teen Imagineering Club
  This code is in the public domain

  Modified by Raymond Blum, added ping sampling, LED and buzzer code,
  abstracted pin assignments and servo orientation
*/
#include <VarSpeedServo.h>  //include the VarSpeedServo library
#include <NewPing.h>        //include the NewPing library
#include <TimerFreeTone.h>

// #define _DEBUG

#define PIN_RIGHT_HIP 9
#define PIN_RIGHT_FOOT 10
#define PIN_LEFT_HIP 5
#define PIN_LEFT_FOOT 6

#define PING_SAMPLES 3
#define AVOIDANCE_DISTANCE_THRESHOLD 8

VarSpeedServo RH;  //create 4 VarSpeedServo objects for the 4 servos
VarSpeedServo RA;
VarSpeedServo LH;
VarSpeedServo LA;

#define PIN_BUZZER 3 // Pin you have speaker/piezo connected to
#define PIN_LED 13

#define PIN_PING_ECHO 4
#define PIN_PING_TRIG 8

NewPing sonar(PIN_PING_TRIG, PIN_PING_ECHO, 200); //create a NewPing object. (trigger pin, echo pin, max distance cm)

const int svsp = 10;  //speed setting for VarSpeedServo SlowMove
const int framedelay = 750;  //delay between frames to allow servo movement

//set the members for the hm array to the home positions for your robot
//they can be found by using the icbob_home_calibration sketch
const int hm[4] = {115, 119, 93, 85}; //array to hold home position for each servo RH,RA,LH,LA

//forward array data
const int fwdmvct = 6;  //Make this number equal to the number of lines in the array
const int fwdmv[fwdmvct][4] =
{
  {0, -40, 0, -20},     //walk forward move frames
  {30, -40, 30, -20},
  {30, 0, 30, 0},
  {0, 20, 0, 40},
  { -30, 20, -30, 40},
  { -30, 0, -30, 0},
};

//turn array data
const int trnmvct = 5;  //Make this number equal to the number of lines in the array
const int trnmv[trnmvct][4] =
{
  { -40, 0, -20, 0},    //turn move frames
  { -40, 30, -20, 30},
  {0, 30, 0, 30},
  {30, 0, 30, 0},
  {0, 0, 0, 0},
};

// Melody (liberated from the toneMelody Arduino example sketch by Tom Igoe).
int melody[] = { 262, 196, 196, 220, 196, 0, 247, 262 };
int duration[] = { 250, 125, 125, 250, 250, 250, 250, 250 };

int lastState;
#define AVOID 1
#define FORWARD 2

void playTune() {
   for (int thisNote = 0; thisNote < 8; thisNote++) { // Loop through the notes in the array.
    TimerFreeTone(PIN_BUZZER, melody[thisNote], duration[thisNote]); // Play melody[thisNote] for duration[thisNote].
    delay(50); // Short delay between notes.
  }
}

void setup()
{
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  RH.attach(PIN_RIGHT_HIP);   //attach the 4 servo objects to the corresponding pins
  RA.attach(PIN_RIGHT_FOOT);  //Robot's right and left when viewed from behind
  LH.attach(PIN_LEFT_HIP);  //Not with the robot facing you!
  LA.attach(PIN_LEFT_FOOT);

  //home servos for 2 seconds to stabilize
  TimerFreeTone(PIN_BUZZER, 500, 500);
  RH.slowmove (hm[0] , svsp);
  LH.slowmove (hm[2] , svsp);
  RA.slowmove (hm[1] , svsp);
  LA.slowmove (hm[3] , svsp);
  delay(3000);
  digitalWrite(PIN_LED, LOW);
  lastState = AVOID;
}

void loop()  //loop repeats forever
{
  //Ping sonar for close encounter
  int tooclose = 0;
  int echoTime = sonar.ping_median(PING_SAMPLES);
  int cm = sonar.convert_cm(echoTime);
  #ifdef _DEBUG
  for (int i=0; i<cm; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(100);
    digitalWrite(PIN_LED, LOW);
    delay(300);
  }
  #endif
  if (cm >= AVOIDANCE_DISTANCE_THRESHOLD)  // If more than 5 cm away, move forward - Else turn
  {
    if (lastState == AVOID) { playTune(); }
    lastState = FORWARD;
    // Do forward move sequence
    for (int x = 0; x < fwdmvct; x++) {                //cycle through the number of
      digitalWrite(PIN_LED, HIGH);
      RH.slowmove (hm[0] + fwdmv[x][0] , svsp);        //lines 'frames' in the array
      RA.slowmove (hm[1] + fwdmv[x][1] , svsp);
      LH.slowmove (hm[2] + fwdmv[x][2] , svsp);
      LA.slowmove (hm[3] + fwdmv[x][3] , svsp);
      digitalWrite(PIN_LED, LOW);
      delay(framedelay);
    }
  }
  else
  {
    lastState = AVOID;
    //Do turn sequence
    digitalWrite(PIN_LED, HIGH);
    for (int z = 0; z < 4; z++) { //repeat sequence 4 times for quarter turn
      for (int y = 0; y < trnmvct; y++) {              //cycle through the number of
        RH.slowmove (hm[0] + trnmv[y][0] , svsp);      //lines 'frames' in the array
        RA.slowmove (hm[1] + trnmv[y][1] , svsp);
        LH.slowmove (hm[2] + trnmv[y][2] , svsp);
        LA.slowmove (hm[3] + trnmv[y][3] , svsp);
        delay(framedelay);
      }
    }
    digitalWrite(PIN_LED, LOW);
  }
}
