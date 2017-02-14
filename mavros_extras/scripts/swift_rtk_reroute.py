#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.client.loggers.json_logger import JSONLogger
from sbp.navigation import SBP_MSG_BASELINE_NED, MsgBaselineNED
import argparse
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import SwiftSbp
from geometry_msgs.msg import PoseStamped
import math

#/**
# * @brief Pack a hil_gps message on a channel
# * @param system_id ID of this system
#* @param component_id ID of this component (e.g. 200 for IMU)
# * @param chan The MAVLink channel this message will be sent over
# * @param msg The MAVLink message to compress the data into
# * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
# * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
# * @param lat Latitude (WGS84), in degrees * 1E7
# * @param lon Longitude (WGS84), in degrees * 1E7
# * @param alt Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
# * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
# * @param epv GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
# * @param vel GPS ground speed (m/s * 100). If unknown, set to: 65535
# * @param vn GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
# * @param ve GPS velocity in cm/s in EAST direction in earth-fixed NED frame
# * @param vd GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
# * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
# * @param satellites_visible Number of satellites visible. If unknown, set to 255
# * @return length of the message in bytes (excluding serial stream start sign)
# */



def binary(num, pre='0b', length=8, spacer=0):
    return '{0}{{:{1}>{2}}}'.format(pre, spacer, length).format(bin(num)[2:])

def gpsrtk():
    #pub = rospy.Publisher('/mavros/gps_reroute/gps_fix', NavSatFix, queue_size=10)
    pub = rospy.Publisher('/mavros/gps_reroute/gps_fix', SwiftSbp, queue_size=10)
    #pub2 = rospy.Publisher('/distance/gps_fix', PoseStamped, queue_size=10)

    rospy.init_node('swift_rtk_reroute', anonymous=True)
    parser = argparse.ArgumentParser(description="Swift Navigation SBP Example.")
    parser.add_argument("-p", "--port",
                      default=['/dev/ttyUSB0'], nargs=1,
                      help="specify the serial port to use.")
    parser.add_argument("-b", "--baud",
                      default=[1000000], nargs=1,
                      help="specify the baud rate to use.")
    args = parser.parse_args()
    # Open a connection to Piksi
    msgFieldCounter = 0 
    with PySerialDriver(args.port[0], args.baud[0]) as driver:
    	with Handler(Framer(driver.read, None, verbose=True)) as source:
		flagMsg1 = False 
		flagMsg2 = False
		flagMsg3 = True
		msg1 = SwiftSbp()
      		try:
        		for msg, metadata in source.filter():
		         # Print out representation of the message
				#msg1 = SwiftSpp()   
			 	if msg.msg_type == 0x0201:    
					#print 'msg1'
					flagMsg1 = True
				  	print msg.lon  * 10000000						
					print msg.lat * 10000000
					print msg.height * 1000
					print 'flag' , msg.flags
					std = bin(msg.flags)
					#print 'reem' , binary(8)
					print 'binary', binary(msg.flags)[7:10]
					br = binary(msg.flags)[7:10]
					print 'int' , int(br)
					msg1.flag= int(br)  
					#print msg.tow
					#print msg.h_accuracy
					#print msg.v_accuracy
					#print msg.n_sats
                                        msg1.header.stamp = rospy.Time.now()
					msg1.latitude = msg.lon * 10000000
					msg1.longitude = msg.lat * 10000000
					msg1.height = msg.height * 1000
					msg1.tow = msg.tow
					msg1. horizontal_accuracy = msg.h_accuracy
					msg1.vertical_accuracy = msg.v_accuracy 
					msg1.numOfSat = msg.n_sats 			

					# Other fields from the message (according to the documentation page 12)
                                        # msg.h_accuracy (mm)
                                        # msg.v_accuracy (mm)
                                        # msg.nsats
                                        # msg.flags - check page 12 of documentation (basicaly: single point, differential, fixed rtk, f)
                                        # msg.tow (time of week in ms)

				

				if msg.msg_type == 0x0205:    
					#print 'msg2'
					flagMsg2 = True
					#print 'x' , msg.n						
					#print 'y' , msg.e
					#print 'z' , msg.d
					msg1.vn = msg.n
					msg1.ve = msg.e
					msg1.vd = msg.d
				 	print 'North' , msg.n  
                                 

				if msg.msg_type == 0x0208:   # or 0x0206 #This msg is never received 
					flagMsg3 = True
					msg1.hdop = msg.hdop						
					msg1.vdop = msg.vdop
				
##################################To calculate, print and publish  the distance between the rover and the base station #################################
				if msg.msg_type == 0x0203:
					dist = math.sqrt(msg.n*msg.n + msg.e*msg.e + msg.d*msg.d) 
					print 'distance in mm' , dist 	
					msg1.baseline_north = msg.n 
					msg1.baseline_east = msg.e
					msg1.baseline_down = msg.d
					msg1.distance  = dist 		


########################################################################################################################################################


					#if msg.msg_type == 0x0200:
					#print "MSG2"
					#msg2 = PoseStamped() 
					#msg2.pose.position.x = msg.x
					#msg2.pose.position.y = msg.y
					#msg2.pose.position.z = msg.z
					#pub2.publish(msg2) 
				
				if(flagMsg1 == True and flagMsg2 == True and flagMsg3 ==True ):
					print "*****************************************"			
					print 'msg3'
					flagMsg1 = False
					flagMsg2 = False
					#flagMsg3 = False
					pub.publish(msg1)
				
      		except KeyboardInterrupt:
        		pass
    #rate = rospy.Rate(10) # 10hz
    #while not rospy.is_shutdown(): 
    #    rate.sleep()

if __name__ == '__main__':
    try:
        gpsrtk()
    except rospy.ROSInterruptException:
        pass
