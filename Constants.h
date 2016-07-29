#ifndef Constants_h
#define Constants_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include <inttypes.h>
#define NOTE_A3  220
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330

#define THR_JOYSTICK A3
#define YAW_JOYSTICK A2
#define ROLL_JOYSTICK A1
#define PITCH_JOYSTICK A0
#define POT A4
#define TOGGLE 2
#define SPEAKER_PIN 3
//#define WAKE_PIN 10

#define YAW_MIN 0
#define YAW_OFFSET 385
#define YAW_MAX 814

#define ROLL_MIN 0
#define ROLL_OFFSET 358
#define ROLL_MAX 796

#define PITCH_MIN 0
#define PITCH_OFFSET 355
#define PITCH_MAX 788

#define THR_MIN 0
#define THR_OFFSET 25
#define THR_MAX 754

const int ESC_MIN = 800;
const int ESC_OFFSET = 1500;
const int ESC_MAX = 2000;
const int ESC_OFF = 767;

#define ANGLE_MIN 22
#define ANGLE_MAX 45
#define RATE_MIN 90
#define RATE_MAX 180

#define RATE_MODE true
#define STABILIZE_MODE false

#define THR_1 0
#define THR_2 1
#define YAW_1 2
#define YAW_2 3
#define ROLL_1 4
#define ROLL_2 5
#define PITCH_1 6
#define PITCH_2 7
#define POT_1 8
#define POT_2 9

#endif
