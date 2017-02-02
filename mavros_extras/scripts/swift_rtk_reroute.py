#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.client.loggers.json_logger import JSONLogger
from sbp.navigation import SBP_MSG_BASELINE_NED, MsgBaselineNED
import argparse
from sensor_msgs.msg import NavSatFix


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
					msg1 = NavSatFix()   
				  	print msg.lon
					print msg.lat
					print msg.height
                                        msg1.header.stamp = rospy.Time.now()
					msg1.latitude = msg.lon
					msg1.longitude = msg.lat
					msg1.altitude = msg.height
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
