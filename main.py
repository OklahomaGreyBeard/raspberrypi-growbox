#!/usr/bin/env python

# Copyright (c) 2015, Kevin Webb
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import argparse
import atexit
import datetime
import sys
import time

import RPi.GPIO
import serial

# TODO:
# fan speed
# Second thread for input

def cleanup(teensy):
    print 'Cleaning up...'
    teensy.close()
    RPi.GPIO.cleanup()

def parse_teensy(serial_string):
    cmd, value = serial_string.split(':')
    value = float(value)

    if cmd == 'TMP':
        value = value * (9.0 / 5.0) + 32

    return cmd, value

def main_loop(args, ontime, offtime):
    RPi.GPIO.setmode(RPi.GPIO.BCM)
    RPi.GPIO.setup(26, RPi.GPIO.OUT)

    teensy = serial.Serial(args.serial, 9600, timeout=1)
    atexit.register(cleanup, teensy)

    RPi.GPIO.output(26, 0)
    teensy.write('PWM0.')
    print parse_teensy(teensy.readline())

    while True:
        teensy.write('TMP.')
        print parse_teensy(teensy.readline())

        now = datetime.datetime.now().time()
        if now > ontime and now < offtime:
            print now, 'Lights on'
            RPi.GPIO.output(26, 1)
        else:
            print now, 'Lights off'

        time.sleep(args.interval)

def compute_schedule(hours):
    if hours >= 24.0:
        print 'No more than 24 hours per day...'
        sys.exit(1)

    hour_span = int(hours / 2.0)
    minute_span = 60.0 * ((hours / 2.0) - hour_span)

    noon = datetime.datetime(1,1,1,12)
    delta = datetime.timedelta(hours=hour_span, minutes=minute_span)

    ontime = (noon - delta).time()
    offtime = (noon + delta).time()

    print 'Light schedule:', ontime, offtime

    return ontime, offtime

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--interval', default=30, dest='interval',
            type=int, help='Time between sensor readings (in seconds). [30]')
    parser.add_argument('-l', '--lights', required=True, type=float,
            help='The number of hours to keep the lights on per day. (float)')
    parser.add_argument('-s', '--serial', default='/dev/ttyACM0',
            help='The name of the serial device. [/dev/ttyACM0]')
    parser.add_argument('-t', '--temperature', default=75, type=float,
            help='The target temperature, in degrees Fahrenheit. (float) [75]')
    args = parser.parse_args()

    ontime, offtime = compute_schedule(args.lights)

    main_loop(args, ontime, offtime)
