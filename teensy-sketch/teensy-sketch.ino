/*

Copyright (c) 2014, Kevin Webb
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

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
