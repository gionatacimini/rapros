#!/usr/bin/env python

#The rapros package is a native ROS-Simulink package, that allows Rapid Prototyping
#task. Rapid Prototyping is the set of procedures which helps to design and to develop 
#control algorithms for robotics applications, it is a general concept which includes 
#both Processor in the Loop and Hardware in the Loop.
#rapros provide an efficient communication between ROS and Simulink environments
#based on UDP.
#
#
# Copyright (c) 2015, Luca Cavanini, Gionata Cimini.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, 
# are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice,
#	 this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation 
#	 and/or other materials provided with the distribution.
#
#3. Neither the name of the copyright holder nor the names of its contributors 
#   may be used to endorse or promote products derived from this software 
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
# IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import numpy as np
import sys, struct
import array
import time
from socket import *
from std_msgs.msg import Float32MultiArray

X_data = Float32MultiArray()#
U_data = Float32MultiArray()#

#The maximum amount of data to receive at a time
MAX_DATA_SIZE = 2048
#The port to send/receive data on
PORT = 667
PORTA = 25001
#Get the IP address of this computer
IPB= gethostbyname('127.0.0.1')
IPP= gethostbyname('127.0.0.1')
# UDP sender
sock_sen = socket(AF_INET, SOCK_DGRAM)

# UDP receiver
sock_rec = socket(AF_INET, SOCK_DGRAM)
#sock_rec.bind((COMPUTER_IP_ADDRESS,PORTA))
#sock_rec.settimeout(3*Ts)




def send_U(dati):
####################### SEND DATA TO SIMULINK ###############	
	out_data=dati.data

	ip = rospy.get_param('IP')
	IPP = ip['ip_pc']
	
	try :
		leng=len(out_data)
		To_Simulink = struct.pack('d'*leng,*out_data)
	except TypeError:
		To_Simulink = struct.pack('d',out_data)
		
	print '.................................'
	sock_sen.sendto(To_Simulink, (IPP, PORT))#to Simulink
	#print "To_Simulink transmitted",To_Simulink
	print "Transmitted to Simulink"
##############################################################



def receive_X():
#######################   RECEIVE DATA FROM SIMULINK ##########
		in_data= {}
		n_input=0
		data_out = Float32MultiArray()#

####################### Control receiving data from Simulink ##
		#	print 'Wait upd packet'
		#	try:
		print 'wait udp pakcet'
		(data,addr) = sock_rec.recvfrom(MAX_DATA_SIZE)
		print 'received'
		#	except timeout:
		#		print '########### ERROR #################'
		#		print '3 UDP package lost. Stop simulation'
		#		return 
		#	print 'Udp packet received'
################################################################
		dim=len(str(data))
		n_input=(dim/8)-1 #8 byte for each signal value sample
####################### UNPACK #################################
		in_data[0]=struct.unpack('<d',data[0:8])
		i=0
		while (i<n_input): 
			i=i+1
			in_data[i]=struct.unpack('<d',data[i*8:i*8+8])

		i=0
		while (i<=n_input):
			IN_data=in_data.get(i)
			data_out.data.append(IN_data[0]);
			i=i+1

		return data_out
################################################################

def Callback(data):
	global To_SIM
	U_data.data = data.data



def RaPROS():

	version=rospy.get_param("/rosdistro").rstrip()


	ts = rospy.get_param('Ts')
	Ts = ts['Ts']
	freq=1/Ts

	ip = rospy.get_param('IP')
	IPB = ip['ip_board']

	rospy.init_node('RaPROS_loopback', anonymous=True)
	rospy.Subscriber("Y", Float32MultiArray, Callback)

	if version[0]<="h" :
		pub = rospy.Publisher('U', Float32MultiArray) ## for ros version before hydro
	else:
		pub = rospy.Publisher('U', Float32MultiArray,queue_size=10) ## for hydro and newest ros versions


	in_data= {}
	
	r = rospy.Rate(freq)

	sock_rec.bind((IPB,PORTA))

	while not rospy.is_shutdown():		

			X_data=receive_X() #receive X state data from simulink

			pub.publish(X_data) #Publishing data to U topic if you need

			send_U(X_data) #send U control data to simulink
##############################################
			X_data.data=[] #clear list after data send


if __name__ == '__main__':
    try:
        RaPROS()
    except rospy.ROSInterruptException: pass


