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
IPB= 0
IPP= 0
# UDP sender
sock_sen = socket(AF_INET, SOCK_DGRAM)

# UDP receiver
sock_rec = socket(AF_INET, SOCK_DGRAM)

#sock_rec.bind((COMPUTER_IP_ADDRESS,PORTA))
#sock_rec.settimeout(3*Ts)

# discrete PID parameters in parallel form##########################################
#PID Z
KpZ=-22.9524857040326
KiZ=-0.366329627958056
KdZ=-215.804435520502
Ts=0.1
NZ=6.32298167776928

a0Z = (1+NZ*Ts);    
a1Z = -(2 + NZ*Ts);
a2Z = 1;
b0Z = KpZ*(1+NZ*Ts) + KiZ*Ts*(1+NZ*Ts) + KdZ*NZ;
b1Z = -(KpZ*(2+NZ*Ts) + KiZ*Ts + 2*KdZ*NZ);
b2Z = KpZ + KdZ*NZ;
ku1Z = a1Z/a0Z; ku2Z = a2Z/a0Z; ke0Z = b0Z/a0Z; ke1Z = b1Z/a0Z; ke2Z = b2Z/a0Z;


###############################################################################################
#To stabilize, reference array [0,0,0,0]
ref=np.zeros([4,1])

#error array
e0=np.zeros([4,1])
e1=np.zeros([4,1])
e2=np.zeros([4,1])

#control signal array
u0=np.zeros([4,1])
u1=np.zeros([4,1])
u2=np.zeros([4,1])

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
	print 'ipp',IPP
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



def rapROS():

	version=rospy.get_param("/rosdistro").rstrip()


	ts = rospy.get_param('Ts')
	Ts = ts['Ts']
	freq=1/Ts


	ip = rospy.get_param('IP')
	IPB = ip['ip_board']

	print 'ipb',IPB

	ip = rospy.get_param('IP')
	IPP = ip['ip_pc']

	print 'ipp',IPP

	rospy.init_node('rapROS_quadrotor', anonymous=True)
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
############################################################
# PID controllers implementation############################
############################################################
			print X_data.data[0]#X
			print X_data.data[1]#Y
			print X_data.data[2]#Z
			print X_data.data[3]#Phi
			print X_data.data[4]#Theta
			print X_data.data[5]#Psy
			print ref# reference array referred do [Z,Theta,Phi,Psy]
			
			#e2=e1; e1=e0; u2=u1; u1=u0;
			#update previuos error
			e2[0]=e1[0]
			e2[1]=e1[1]
			e2[2]=e1[2]
			e2[3]=e1[3]

			e1[0]=e0[0]
			e1[1]=e0[1]
			e1[2]=e0[2]
			e1[3]=e0[3]

			#calculate last error
			e0[0]=X_data.data[2]#Z error
			e0[1]=X_data.data[3]#Phi error
			e0[2]=X_data.data[4]#Theta error
			e0[3]=X_data.data[5]#Psy error

			#update previuos control signals
			u2[0]=u1[0]
			u2[1]=u1[1]
			u2[2]=u1[2]
			u2[3]=u1[3]

			u1[0]=u0[0]
			u1[1]=u0[1]
			u1[2]=u0[2]
			u1[3]=u0[3]

			#calculate last control signals
			u0[0] = -ku1Z*u1[0] - ku2Z*u2[0] + ke0Z*e0[0] + ke1Z*e1[0] + ke2Z*e2[0]

			U_data.data=u0

############################################################
			pub.publish(X_data) #Publishing data to U topic if you need

			send_U(U_data) #send U control data to simulink

##############################################
			X_data.data=[] #clear list after data send
			U_data.data=[]
			



if __name__ == '__main__':
    try:
        rapROS()
    except rospy.ROSInterruptException: pass


