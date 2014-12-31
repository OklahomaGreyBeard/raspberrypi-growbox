#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <Wire.h>

#include "Adafruit_AM2315.h"

#define CMDBUFFLEN (16)
#define CMDLEN (3)

int pwmPin = 14;
int ledPin = 11;

char delimiter = '.';
char cmdbuff[CMDBUFFLEN];
int cmdpos = 0;

Adafruit_AM2315 am2315;

void led_blink(int howmany, int delayms) {
  int i;
  for (i = 0; i < howmany; ++i) {
    delay(delayms);
    digitalWrite(ledPin, HIGH);
    delay(delayms);
    digitalWrite(ledPin, LOW);
  }
}

void setup() {
  memset(cmdbuff, 0, CMDBUFFLEN);
  pinMode(pwmPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);

  delay(2000);

  if (!am2315.begin()) {
    Serial.println("ERR:Sensor Missing");
    led_blink(3, 1500);
  } else {
    led_blink(2, 300);
  }
}

int readCommand() {
  while (Serial.available()) {
    char b = Serial.read();

    if (b == delimiter) {
      cmdbuff[cmdpos] = '\0';
      cmdpos = 0;
      return 1;
    } else {
      cmdbuff[cmdpos] = b;
      cmdpos += 1;
    }
  }
  return 0;
}

int setPWM() {
  int val = atoi(cmdbuff + CMDLEN);

  if (val > 255)
    val = 255;
  if (val < 0)
    val = 0;

  analogWrite(pwmPin, val);
  return val;
}

void loop() {
  delay(500);
  if (readCommand()) {
    if (!strncmp(cmdbuff, "PWM", CMDLEN)) {
      Serial.print("PWM:");
      Serial.println(setPWM());
    } else if (!strncmp(cmdbuff, "TMP", CMDLEN)) {
      Serial.print("TMP:");
      Serial.println(am2315.readTemperature());
    } else if (!strncmp(cmdbuff, "HUM", CMDLEN)) {
      Serial.print("HUM:");
      Serial.println(am2315.readHumidity());
    } else {
      Serial.println("ERR:Unknown command");
    }
  }
}
