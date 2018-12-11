
# *****************************************************************************
# Copyright (c) 2017 IBM Corporation and other Contributors.
#
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v1.0
# which accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
#   Hari hara prasad Viswanathan - Initial Contribution
# *****************************************************************************

import getopt
import time
import sys
import uuid
import threading
import time
import signal
import os
import random
import ibmiotf.device
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(13, GPIO.OUT)

import serial

#initialize serial port uart
serialport = serial.Serial( 
port='/dev/ttyAMA0',
baudrate=9600,
parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_TWO,
bytesize=serial.EIGHTBITS)


# Variables
motor_up_time = time.time()
running_status = False
global keepUploadingData
keepUploadingData=True


def usage():
    print(
        "commandSender: Raspberry Pi-powered conveyor belt" + "\n" +
        "\n" +
        "Options: " + "\n" +
        "  -h, --help    Display help information" + "\n" +
        "  -t, --time    [Mandatory] Time period in minutes to stop" +
        " the motor, Recommended maximum value is 5" + "\n"
        )


class set_interval():  # Timer wrapper
    def __init__(self, func, sec):
        def func_wrapper():
            self.t = threading.Timer(sec, func_wrapper)
            self.t.start()
            func()
        self.t = threading.Timer(sec, func_wrapper)
        self.t.start()

    def cancel(self):
        self.t.cancel()


def start_handler():  # Turn on motor connected in pin 13
        global motor_up_time
        global running_status
        motor_up_time = time.time()
        running_status = True
        GPIO.output(13, GPIO.HIGH)
        print("starting mototr")


def stop_handler():  # Turn off motor connected in pin 13
        global motor_up_time
        global running_status
        motor_up_time = time.time()
        running_status = False
        GPIO.output(13, GPIO.LOW)
        print("Stoping the motor")


def my_command_callback(cmd):  # Handle device command
    print("Command received: %s" % cmd.command)
    if cmd.command == "stop":
        stop_handler()
    if cmd.command == "start":
        start_handler()


def my_on_publish_callback():
    print("Confirmed event  received by WIoTP\n")


def get_cpu_temperature():  # Return CPU temperature as a character string
    res = os.popen('vcgencmd measure_temp').readline()
    return(res.replace("temp=", "").replace("'C\n", ""))



def get_rpm():  # Return current rpm value
    # we are generating a random value here to represent motor's r.p.m
    if not running_status:
        return 0.0
    delta = -0.1
    if (time.time() % 2) == 0:
        delta = 0.1
    rmp = random.uniform(1, 3.1) + delta
    return round(rmp, 1)


def get_ay():  # Return accelerometer value
    # we are generating a random value here to represent accelerometer
    delta = -0.05
    if (time.time() % 2) == 0:
        delta = 0.05
    ay = random.uniform(-1.0, 1.0) + delta
    return round(ay, 3)


def publish(rx_data_str): #use rx_str as parameter to get the information into json format
        listOfStrings = rx_data_str.split()
        if listOfStrings[0]==1:
            listOfStrings[0]="up"
        else
            listOfStrings[0]="down"
        listOfStrings[1] = "disconnected" #that directional motor is disconnected as of now
        state = "false"
        if running_status:
            state = "true"
        data = {'eastLDR': listOfStrings[5],
                'northLDR': listOfStrings[2],
                'westLDR': listOfStrings[4],
                'tiltDir': listOfStrings[0],
                'southLDR': listOfStrings[3]}
        success = device_client.publishEvent(
            "sensorData",
            "json",
            {'d': data},
            qos=0,
            on_publish=my_on_publish_callback)
        listOfStrings.clear()
        if not success:
            print("Not connected to WIoTP")


def signal_handler(signal, frame):
    print ('You pressed Ctrl+C!')
    keepUploadingData=False
    timeout.cancel()
    GPIO.cleanup()
    # Disconnect the device from the cloud
    device_client.disconnect()


def stop_and_exit():
    stop_handler()
    timeout.cancel()
    GPIO.cleanup()
    device_client.disconnect()
    sys.exit()

if __name__ == "__main__":
    stopTime = 5.0
    try:
        opts, args = getopt.getopt(sys.argv[1:], "ht:", ["help", "time="])
        if len(opts) == 0:
            print("Input argument missing")
            GPIO.cleanup()
            usage()
            sys.exit()
        for o, a in opts:
            if o in ("-t", "--time"):
                stopTime = float(a)
                if stopTime > 5.0:
                    print(
                            "WARNING : Motor stop time period you have set" +
                            " is more than recommended value " +
                            "(5 min),However the motor will run " +
                            "till the time you have set"
                          )
            elif o in ("-h", "--help"):
                GPIO.cleanup()
                usage()
                sys.exit()
            else:
                print("unhandled option" + o)

    except getopt.GetoptError as err:
        print(str(err))
        GPIO.cleanup()
        usage()
        sys.exit(2)
    # Initialize the device client.
    try:
        device_file = "device.conf"
        device_options = ibmiotf.device.ParseConfigFile(device_file)
        device_client = ibmiotf.device.Client(device_options)
    except Exception as e:
            print("Caught exception connecting device: %s" % str(e))
            GPIO.cleanup()
            sys.exit()

    device_client.connect()
    device_client.commandCallback = my_command_callback

    # timeout = set_interval(publish, 1)
    # start_handler()
    # print ("Motor will stop in %s min" % str(stopTime))
    # r = threading.Timer(stopTime*60.0, stop_and_exit)
    # r.start()

    signal.signal(signal.SIGINT, signal_handler) #maybe this allows the program to exit when user press ctr+c

    while keepUploadingData: #read uart_rx_pin and send data to cloud
        rx_uart_data = serialport.readline() #blocking read on uart port_pin
        rx_uart_data = str(rx_uart_data)
        rx_uart_data = rx_uart_data[:-1] #remove trailing newline from received string
        publish(rx_uart_data) #send data to cloud

    # signal.signal(signal.SIGINT, signal_handler)
    serialport.close()
print ('Press Ctrl+C to exit')