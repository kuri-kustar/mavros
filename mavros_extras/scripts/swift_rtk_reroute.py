#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.client.loggers.json_logger import JSONLogger
from sbp.navigation import SBP_MSG_BASELINE_NED, MsgBaselineNED
import argparse
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import SwiftSpp


def gpsrtk():
    pub = rospy.Publisher('/mavros/gps_reroute/gps_fix', NavSatFix, queue_size=10)
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
    with PySerialDriver(args.port[0], args.baud[0]) as driver:
    	with Handler(Framer(driver.read, None, verbose=True)) as source:
      		try:
        		for msg, metadata in source.filter():
		         # Print out representation of the message	
			 	if msg.msg_type == 0x0201:    

					msg1 = SwiftSpp()   
				  	print msg.lon 						
					print msg.lat
					print msg.height
					print msg.tow
					print msg.h_accuracy
					print msg.v_accuracy
					print msg.n_sats
                                        msg1.header.stamp = rospy.Time.now()
					msg1.latitude_s = msg.lon * 10000000
					msg1.longitude_s = msg.lat * 10000000
					msg1.height_s = msg.height * 1000
					msg1.tow_s = msg.tow
					msg1.horizontal_acc_s = msg.h_accuracy
					msg1.vertical_acc_s = msg.v_accuracy 
					msg1.number_of_Sat = msg.n_sats 			
					pub.publish(msg1)
					'''
					msg1 = NavSatFix()   
				  	print msg.lon 
					print msg.lat
					print msg.height
                                        msg1.header.stamp = rospy.Time.now()
					msg1.latitude = msg.lon * 10000000
					msg1.longitude = msg.lat * 10000000
					msg1.altitude = msg.height * 1000
					pub.publish(msg1)
					'''
				if msg.msg_type == 0x0202:    
					print msg.x						
					print msg.y
					print msg.z
					msg1.x_vector_from_base_to_rover = msg.x
					msg1.y_vector_from_base_to_rover = msg.y
					msg1.z_vector_from_base_to_rover = msg.z


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
