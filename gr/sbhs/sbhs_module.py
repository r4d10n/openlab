#/usr/bin/env python

from gnuradio import gr
import serial
import threading
import time
import numpy


# Hardware addresses for Single Board Heater System components
FAN_ADDR = 253
HEATER_ADDR = 254
TEMPSENSOR_ADDR = 255

class sbhs_module(gr.hier_block2, threading.Thread):
    def __init__(self, samp_rate, fan_speed, heater_temp):    
        gr.hier_block2.__init__(self, 'sbhs_module', gr.io_signature(0,0,0), gr.io_signature(1,1,gr.sizeof_float))

	self.init_serial()

        self.set_samp_rate(samp_rate)
	self.set_fan_speed(fan_speed)
	self.set_heater_temp(heater_temp)

        message_source = gr.message_source(gr.sizeof_int,1)
        self._msgq = message_source.msgq()
        self.connect(message_source, self)

        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.start()
        
    def run(self):
        while True:
            time.sleep(1.0/self._samp_rate)
    
            temp = self.get_temp_from_sensor()
            self._msgq.insert_tail(temp)

    def write_serial(self, hw_addr, val):
	self._serial.write(chr(hw_addr))
	self._serial.write(chr(val))

    def init_serial(self):
	self._serial = serial.Serial('/dev/ttyUSB0', 9600)
	self._serial.open()
            
    def get_temp_from_sensor(self):
	self._serial.write(chr(TEMPSENSOR_ADDR))
	temp_val = map(ord, self._serial.read(2))
	temp_string = str(temp_val[0]) + str(temp_val[1])	
        arr = numpy.array(float(temp_string)/10.0, numpy.float32)
	print "Read Temperature from Sensor: %s" %(arr)
        return gr.message_from_string(arr.tostring(), 0, gr.sizeof_float, 1)
            
    def set_samp_rate(self, samp_rate):
        self._samp_rate = samp_rate
            
    def set_fan_speed(self, fan_speed):
	self._fan_speed = fan_speed
	print "Setting Fan Speed: %d" %(fan_speed)
        self.write_serial(FAN_ADDR, self._fan_speed)

    def	set_heater_temp(self, heater_temp):
	self._heater_temp = heater_temp
	print "Setting Heater Temp: %d" %(heater_temp)
        self.write_serial(HEATER_ADDR, self._heater_temp)
	

