"""
Communicates with several arduinos to command them and collect data over I2C.
We will be converting ints, floats, and strings to ascii values and then sending it over to the Arduinos. Additonally, we will be receiving ascii-values and converting them to ints, floats, or strings as needed.


More about the smbus library can be found here: http://wiki.erazor-zone.de/wiki:linux:python:smbus:doc
"""

import smbus
import time

bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

def writeNumber(value):
    bus.write_byte(address, value)
    return -1

def readNumber():
    number = bus.read_byte(address)
    # number = bus.read_byte_data(address, 1)
    return chr(number)

while True:
    var = raw_input("Enter 1 - 9: ")
    
    if not var:
        continue
    for char in var:
        print(char)
        char_ascii = ord(char)
        writeNumber(char_ascii)
        
    print("RPI: Hi Arduino, I sent you ", var)
    # sleep one second
    time.sleep(1)
    
    char_a = readNumber()
    word = ""
    while(char_a != '/0'):
        word += ""
    print("Arduino: Hey RPI, I received a digit", word)
