#!/usr/bin/python

# Wireless Virtual Lab Server v5.2 - Websockets Server
#
# Requires matplotlib, numpy, pyserial, pyrtlsdr, tornado
#
# Uses BG7TBL Signal Generator and PTS-306 CCTV Rotor (1 RPM - AC) [Relay Control with Arduino - see RotorRelay.ino]
# Modify SigGen and Rotor Port/Baud settings as suited.

import sys
import time

from matplotlib.mlab import psd
import numpy as np
from numpy import array

import serial
from rtlsdr import RtlSdr

import tornado.httpserver
import tornado.websocket
import tornado.ioloop
from tornado.ioloop import PeriodicCallback
import tornado.web
from tornado.escape import json_encode


wsPort = 9000 #Websocket Port
timeInterval= 500 #Milliseconds

NFFT = 1024*4
NUM_SAMPLES_PER_SCAN = NFFT*16
NUM_BUFFERED_SWEEPS = 100
NUM_SCANS_PER_SWEEP = 1
FINAL_BINS = 16

SIGGENPORT = "COM3"
SIGGENBAUD = "57600"

ROTORPORT = "COM8"
ROTORBAUD = "9600"


class WSHandler(tornado.websocket.WebSocketHandler):    
    def check_origin(self, origin):
        return True

    def open(self):        
        self.siggen = SigGen()
        self.sdr = RtlSdr()    
        self.scan = Scan(self.sdr)
        self.rotor = Rotor()
        self.callback = PeriodicCallback(self.send_values, timeInterval)
        

    def send_values(self):
        print "::WS: Begin scanning...."
        scan_vals = self.scan.scan()
        
        print "::WS: Detecting Signal Peaks...."
        peaks = detect_peaks(scan_vals)
        
        print "::WS: Sending peak vals to WS...."
        self.write_message("++")
        self.write_message(json_encode(peaks))

    def on_message(self, message):

        print ":: Recvd message: %s" % message
        
        if (message.startswith("==Frequency:")):
                
                str = message.split("|");
                freq = str[0].split(" ")[1]
                rxgain = str[1].split(" ")[1]
                
                print "::WS: Setting Tx Freq:" + freq
                self.siggen.setTxFreq(freq)
                
                print "::WS: Setting Rx Freq:" + freq + "| RxGain: " + rxgain
                self.scan.setFreqGain(freq, rxgain)
                
                print "::WS: Initializing Callback function..."
                self.callback.start()			
                
        elif (message.startswith("==TxOff")):
                print "::WS: Resetting Callback"
                self.callback.stop()
                
                print "::WS: Setting Tx Off"			
                self.siggen.setTxOff()
                
                print "::WS: Closing SDR instance"
                #self.sdr.close()
				
        elif (message.startswith("==Rotor:")):   # ==Rotor: R|2000
			
                str = message.split("|");
                dir = str[0].split(" ")[1]
                deg = str[1]
                self.rotor.setRotor(dir,deg)
        
                        

    def on_close(self):
        print "::WS: Resetting Callback"
        self.callback.stop()
                
        print "::WS: Setting Tx Off"			
        self.siggen.setTxOff()
        
        print "::WS: Closing SDR instance"
        self.sdr.close()
        self.rotor.close();
        self.siggen.close();

class Rotor(object):                    # 1 RPM AC CCTV Rotor PTS-306	
	def __init__(self):
		self.ser = serial.Serial(ROTORPORT,ROTORBAUD, timeout=1)
		if(self.ser.isOpen() == False):
			self.ser.open()
		print (":: Rotor Initializing... ")
		print self.ser.readline()
		print self.ser.readline()

		
	def setRotor(self, dir, deg):
		
		if(self.ser.isOpen() == False):
			self.ser.open()	
			
		print self.ser.readline()
		secs = float(deg) * 160;		
		self.ser.write(str(dir) + ":" + str(int(secs)) +";")		
		print self.ser.readline()

	def close(self):
		self.ser.close();

class SigGen(object):                   # BG7TBL Signal Generator 
	def __init__(self):
		self.ser = serial.Serial(SIGGENPORT,SIGGENBAUD, timeout=1)
		print (":: SigGen version Check....")
		self.ser.write("\x8f"+"v")		
		print ord(self.ser.readline())
		
	def setTxFreq(self,freq):
		
		if(self.ser.isOpen() == False):
			self.ser.open()
			
		if (float(freq) < 35 and float(freq) > 1600):
			print "::SigGen:# Frequency out of coverage band"
			return "::# Frequency out of coverage band"
		else:
			freqstr = "{0:0>9}".format(int(float(freq+"e5")))
			self.ser.write("\x8f\x73\x8f"+ "f"+ freqstr)
			print self.ser.readline()
			self.ser.write("\x8f\x76\x8f"+ "f"+ freqstr)
			print self.ser.readline()
			print "::SigGen: Tuning to frequency: " + str(freq) + "MHz"
			return "::SigGen: Tuning to frequency: " + str(freq) + "MHz"
	
	def setTxOff(self):
	
		if(self.ser.isOpen() == False):
			self.ser.open()
	
		freqstr = "001000000"
		self.ser.write("\x8f\x72\x00\x8f\x72\x00\x8f\x72\x00"+ "f"+ freqstr)
		print "::SigGen: Setting TX off"
		self.ser.close()
		return "::SigGen: Setting TX off"
	    
	def close(self):
		self.ser.close();		


class Scan(object):

    def __init__(self, sdr=None):  
        print "::Scan: Initializing SDR..."
        self.sdr = sdr if sdr else RtlSdr()
       		
    def setFreqGain(self, freq, rxgain):
        self.sdr.fc = float(freq + "e6")
        self.sdr.gain = float(rxgain)
	    
    def scan(self):
        samples = self.sdr.read_samples(NUM_SAMPLES_PER_SCAN)
        psd_scan, f = psd(samples, NFFT=NFFT)
        return 10*np.log10(psd_scan)

def detect_peaks(scan_vals):
    max_peaks = []
    max_value = scan_vals[0];
    max_count = len(scan_vals) / FINAL_BINS

    for i in range(len(scan_vals)):
        if (i % max_count == 0):
            if (i != 0): max_peaks.append(int(max_value))
            max_value = scan_vals[i]            
        else:
            max_value = max(max_value, scan_vals[i])
            
    return max_peaks

application = tornado.web.Application([
    (r'/', WSHandler),
])

if __name__ == '__main__':
    try:
        print "[Wireless Virtual Lab Server v5.2] - Serving at ws: " + str(wsPort)

        http_server = tornado.httpserver.HTTPServer(application)
        http_server.listen(wsPort)
        tornado.ioloop.IOLoop.instance().start()    
        
    except KeyboardInterrupt:
        self.sdr.close()
        self.rotor.close();
        self.siggen.close();

        print "Bye"
        sys.exit()

    
