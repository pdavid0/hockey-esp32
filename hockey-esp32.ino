#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include "driver/timer.h"
#include "pitches.h"
#include "esp_log.h"

#define TIMER_RESOLUTION_HZ   1000000 // 1MHz resolution
#define TIMER_ALARM_PERIOD_S  0.5     // Alarm period 0.5s

#define RED_LED_PIN  12
#define RED_BTN_PIN  13

#define BLUE_LED_PIN  18
#define BLUE_BTN_PIN  19
#define SOUND_PIN  15

// notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

int introMelody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

int goalMelody[] = {
  NOTE_A4, NOTE_C3, NOTE_E3, NOTE_G3, NOTE_E3, NOTE_G3, 0, 0
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  8, 8, 8, 8, 4, 8, 4, 8, 8
};

Adafruit_AlphaNum4 scoreDisplay = Adafruit_AlphaNum4();

uint8_t counter = 0;

// pin definitions:
const int screenPin = 0x70;   // i2c address of the screen

// setting PWM properties
const int freq = 5000;
const int piezoChannel = 0;
const int resolution = 8;

// score management
int scoreRed[2] = {0, 0};
int scoreBlue[2] = {0, 0};
int buttonRedState = 0;
int buttonBlueState = 0;
bool gameStarted = false;

// time
String text = "0500";
int secondsLeft = 0;
hw_timer_t * timer = NULL;
volatile int interruptCounter;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void playMelody(int *melody) {
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 8; thisNote++) {
    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    //    tone(8, melody[thisNote], noteDuration);
    ledcWriteTone(piezoChannel, melody[thisNote]);
    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.3;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    ledcWriteTone(piezoChannel, 0);
  }
}

//
// reverse array access, 1 is in fact 1,0
// int = [1,3];
//  --->  31|
void addGoal(int teamScore[], int count, int pin) {
  int digit0, digit1;
  digit0 = teamScore[1];
  digit1 = teamScore[0];
  if (digit0 + count >= 10) {
    digit0 = 10 - (digit0 + count);
    digit1 = digit1 + 1;
  } else {
    digit0 = digit0 + count;
  }
  teamScore[1] = digit0;
  teamScore[0] = digit1;

  printScore();

  scoreDisplay.blinkRate(HT16K33_BLINK_2HZ);
  digitalWrite(pin, HIGH);
  playMelody(goalMelody);
  delay(300);
  digitalWrite(pin, LOW);
  delay(300);
  digitalWrite(pin, LOW);
  delay(300);
  digitalWrite(pin, LOW);
  scoreDisplay.blinkRate(HT16K33_BLINK_OFF);
}

// TODO: split score to char
void printScore() {
  Serial.println(scoreRed[0], scoreRed[1]);

  scoreDisplay.clear();
  scoreDisplay.writeDigitAscii(0, (int)String(scoreRed[0]).charAt(0));
  scoreDisplay.writeDigitAscii(1, (int)String(scoreRed[1]).charAt(0));
  scoreDisplay.writeDigitAscii(2, (int)String(scoreBlue[0]).charAt(0));
  scoreDisplay.writeDigitAscii(3, (int)String(scoreBlue[1]).charAt(0));
  scoreDisplay.writeDisplay();
}

// Code with critica section
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}
//https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/

// connect sensort and gizmos
// + ledc => esp32 sound;=
// + scoreDisplay => arduino => score display matrixes
// + button =>
// play game start sound
void setup() {
  Serial.begin(115200);
  Serial.println("Hockey !");

  // configure LEDs
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);

  // configure sound, attach channel
  ledcSetup(piezoChannel, freq, resolution);
  ledcAttachPin(SOUND_PIN, piezoChannel);

  // initialize timer
  interruptCounter = 0;

  // initialize the buttons pin as an input:
  pinMode(RED_BTN_PIN, INPUT);
  pinMode(BLUE_BTN_PIN, INPUT);

  // display
  scoreDisplay.begin(screenPin);
  //   scoreDisplay.setBrightness(8);
  playMelody(introMelody);
  // show 0.0.|0.0. on the score display
  printScore();
}

void startGame() {
  Serial.println("Start Game ->" + text + "minutes" );
  secondsLeft = 5 * 60;
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_RESOLUTION_HZ, true);
  timerAlarmEnable(timer);
}

void onTimeOut() {
  Serial.println("Game Over");

  portENTER_CRITICAL(&timerMux);
  gameStarted = false;
  secondsLeft = 0;
  interruptCounter = 0;
  portEXIT_CRITICAL(&timerMux);
}

void appendMinutes() {
  text = "";
  int v = secondsLeft / 60;
  if (v < 10) {
    text += "0";
  }
  text += v;
}

void appendSeconds() {
  text += ":";
  int v = secondsLeft % 60;
  if (v < 10) {
    text += "0";
  }
  text += v;
}

void calculateRemainingAndShow() {
  appendMinutes();
  appendSeconds();
  scoreDisplay.clear();
  scoreDisplay.writeDigitAscii(0, (int)text[0]);
  scoreDisplay.writeDigitAscii(1, (int)text[1]);
  scoreDisplay.writeDigitAscii(2, (int)text[3]);// skip colon
  scoreDisplay.writeDigitAscii(3, (int)text[4]);
  scoreDisplay.writeDisplay();
}

void loop() {
  buttonRedState = digitalRead(RED_BTN_PIN);
  buttonBlueState = digitalRead(BLUE_BTN_PIN);
  // todo: save score.
  if (buttonRedState == HIGH && buttonBlueState == HIGH) {
    if (gameStarted == false) {
      gameStarted = true;

      scoreRed[0] = 0;
      scoreRed[1] = 0;

      scoreBlue[0] = 0;
      scoreBlue[1] = 0;

      startGame();
    }
  } else {
    // check button pressed
    if (buttonRedState == HIGH) {
      Serial.println("Red score" + String(buttonRedState));
      addGoal(scoreRed, 1, RED_LED_PIN);
    } else {
      digitalWrite(RED_LED_PIN, LOW);
    }

    if (buttonBlueState == HIGH) {
      Serial.println("Blue score" + String(buttonBlueState));
      addGoal(scoreBlue, 1, BLUE_LED_PIN);
    } else {
      digitalWrite(BLUE_LED_PIN, LOW);
    }
  }

  //Serial.println("seconds: " + String(secondsLeft) + ", counter: " + String(interruptCounter) + ", time: " + text);

  if (gameStarted) {
    scoreDisplay.clear();
    scoreDisplay.writeDigitAscii(0, (int)text[0]);
    scoreDisplay.writeDigitAscii(1, (int)text[1]);
    scoreDisplay.writeDigitAscii(2, (int)text[3]);// skip colon
    scoreDisplay.writeDigitAscii(3, (int)text[4]);
    scoreDisplay.writeDisplay();
  }
  if (secondsLeft > 0 && interruptCounter > 0) {
    portENTER_CRITICAL(&timerMux);
    secondsLeft -= interruptCounter;
    interruptCounter = 0;
    portEXIT_CRITICAL(&timerMux);
    calculateRemainingAndShow();

    if (secondsLeft <= 0) {
      onTimeOut();
    }
  }
  counter++;
  delay(100);
}
