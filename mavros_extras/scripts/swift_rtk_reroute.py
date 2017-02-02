#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.client.loggers.json_logger import JSONLogger
from sbp.navigation import SBP_MSG_BASELINE_NED, MsgBaselineNED
import argparse
from sensor_msgs.msg import NavSatFix

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
                                        print msg.lon # deg
                                        print msg.lat # deg
                                        print msg.height # meters
                                        msg1.header.stamp = rospy.Time.now()
					msg1.latitude = msg.lon
					msg1.longitude = msg.lat
					msg1.altitude = msg.height
                                        # Other fields from the message (according to the documentation page 12)
                                        # msg.h_accuracy (mm)
                                        # msg.v_accuracy (mm)
                                        # msg.nsats
                                        # msg.flags - check page 12 of documentation (basicaly: single point, differential, fixed rtk, f)
                                        # msg.tow (time of week in ms)
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
