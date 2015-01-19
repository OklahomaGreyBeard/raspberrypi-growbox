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
import readline
import sys
import threading

import RPi.GPIO
import serial

LIGHTS_PIN = 26

def print_readline(text):
    sys.stdout.write('\r' + ' ' * (len(readline.get_line_buffer()) + 2) + '\r')
    print text
    sys.stdout.write('> ' + readline.get_line_buffer())
    sys.stdout.flush()

# TODO: command to print current settings
class Settings(object):
    def __init__(self, args):
        self.interval = args.interval
        self.serial = args.serial
        self.target_temp = args.temperature
        self.schedule_lights(args.lights)

    def schedule_lights(self, hours, offset=0):
        if hours >= 24.0:
            print 'No more than 24 hours per day...'
            return

        if ((hours / 2.0) + offset) > 12:
            print 'Offset doesn\'t fit within a day'
            return

        hour_span = int(hours / 2.0)
        minute_span = 60.0 * ((hours / 2.0) - hour_span)

        noon = datetime.datetime(1,1,1,12 + offset)
        delta = datetime.timedelta(hours=hour_span, minutes=minute_span)

        self.lights_on = (noon - delta).time()
        self.lights_off = (noon + delta).time()

        print_readline('New light schedule:')
        print_readline('  on: %s,  off: %s'%(self.lights_on.strftime('%H:%M'),
                                             self.lights_off.strftime('%H:%M')))


def cleanup(teensy):
    print 'Cleaning up...'
    teensy.close()
    RPi.GPIO.cleanup()

def parse_teensy(serial_string, printit=False):
    cmd, value = serial_string.split(':')

    name, value, suffix = {
        'TMP': ('Temperature', float(value) * (9.0 / 5.0) + 32, 'F'),
        'HUM': ('Humidity', float(value), '%'),
        'PWM': ('Fan speed', float(value) * 100.0 / 255.0, '%')
    }[cmd]

    if printit:
        print '%s: %.2f %s' % (name, value, suffix)

    return name, value, suffix

def periodic_loop(settings, lock, wakeup):
    lock.acquire()

    while True:
        now = datetime.datetime.now().time()

        teensy.write('TMP.')
        tempname, temp, tempsuffix = parse_teensy(teensy.readline())

        # TODO: factor this out so that it's not a duplicate of handle_fans
        temp_diff = temp - settings.target_temp
        percentage = 10 * (temp_diff / 0.25)
        if percentage < 0.0:
            percentage = 0.0
        if percentage > 100.0:
            percentage = 100.0

        pwm = int(percentage * 255.0 / 100.0)
        teensy.write('PWM%d.' % pwm)
        pwmname, percentage, pwmsuffix = parse_teensy(teensy.readline())

        if now > settings.lights_on and now < settings.lights_off:
            RPi.GPIO.output(LIGHTS_PIN, 1)
            lights = 'on'
        else:
            RPi.GPIO.output(LIGHTS_PIN, 0)
            lights = 'off'

        print_readline('%s: %s: %.2f %s, lights: %s, %s: %.2f %s' %
                       (now.strftime('%H:%M'),
                        tempname, temp, tempsuffix,
                        lights,
                        pwmname, percentage, pwmsuffix))

        wakeup.wait(settings.interval)

def handle_exit(settings, args, wakeup):
    sys.exit(0)

def handle_fans(settings, args, wakeup):
    percentage = float(args.split()[0])
    if percentage < 0:
        percentage = 0
    if percentage > 100:
        percentage = 100

    pwm = int(percentage * 255.0 / 100.0)

    teensy.write('PWM%d.' % pwm)
    parse_teensy(teensy.readline(), True)

def handle_hum(settings, args, wakeup):
    teensy.write('HUM.')
    parse_teensy(teensy.readline(), True)

def handle_interval(settings, args, wakeup):
    settings.interval = float(args.split()[0])
    wakeup.notify()

def handle_hours(settings, args, wakeup):
    args = args.split()
    if len(args) == 1:
        settings.schedule_lights(float(args[0]))
    elif len(args) == 2:
        settings.schedule_lights(float(args[0]), int(args[1]))
    else:
        return
    wakeup.notify()

def handle_target(settings, args, wakeup):
    settings.target_temp = float(args.split()[0])
    wakeup.notify()

def handle_temp(settings, args, wakeup):
    teensy.write('TMP.')
    parse_teensy(teensy.readline(), True)

def handle_wakeup(settings, args, wakeup):
    wakeup.notify()

handlers = {
    'exit':        handle_exit,
    'fans':        handle_fans,
    'hours':       handle_hours,
    'hum':         handle_hum,
    'humidity':    handle_hum,
    'interval':    handle_interval,
    'quit':        handle_exit,
    'target':      handle_target,
    'target_temp': handle_target,
    'temp':        handle_temp,
    'temperature': handle_temp,
    'wakeup':      handle_wakeup
}

def rep_loop(settings, lock, wakeup):
    while True:
        user_input = raw_input('> ')
        if len(user_input.strip()) == 0:
            continue

        try:
            cmd, args = user_input.split(None, 1)
        except ValueError:
            cmd, args = user_input, None

        if cmd not in handlers:
            print 'No such command: %s' % cmd
            continue

        lock.acquire()
        handlers[cmd.lower()](settings, args, wakeup)
        lock.release()

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

    # Create initial settings based on command line arguments.
    settings = Settings(args)

    # Initialize the GPIO pin that controls the lights.
    RPi.GPIO.setmode(RPi.GPIO.BCM)
    RPi.GPIO.setup(LIGHTS_PIN, RPi.GPIO.OUT)
    RPi.GPIO.output(LIGHTS_PIN, 0)

    # Initialize serial communication with the Teensy.
    teensy = serial.Serial(settings.serial, 9600, timeout=1)
    teensy.write('PWM0.')
    parse_teensy(teensy.readline(), True)

    # Register a function to clean up on exit.
    atexit.register(cleanup, teensy)

    # Create a thread for user input / control.
    lock = threading.Lock()
    wakeup = threading.Condition(lock)
    t = threading.Thread(target=periodic_loop, args=(settings, lock, wakeup))
    t.daemon = True
    t.start()

    # Drop into a permanent periodic loop.
    rep_loop(settings, lock, wakeup)
