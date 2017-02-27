#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.client.loggers.json_logger import JSONLogger
from sbp.navigation import SBP_MSG_BASELINE_NED, MsgBaselineNED
import sbp.version
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

def gpsrtk():

    rospy.sleep(0.5)  # wait for a while for init to complete before printing
    rospy.loginfo(rospy.get_name() + " start")
    rospy.loginfo("libsbp version currently used: " + sbp.version.get_git_version())

    serial_port = rospy.get_param('~serial_port', '/dev/piksi')
    baud_rate   = rospy.get_param('~baud_rate', 1000000)
    
    print serial_port,baud_rate
    
    pub = rospy.Publisher('/mavros/gps_reroute/gps_fix', SwiftSbp, queue_size=10)
    rospy.init_node('swift_rtk_reroute', anonymous=True)
    # We might need it later
    poseStampedECEF = PoseStamped()
    # Open a connection to Piksi
    with PySerialDriver(serial_port, baud_rate) as driver:
        with Handler(Framer(driver.read, None, verbose=True)) as source:
            poseLLHReceived = False
            poseNEDReceived = False
            velNEDReveived  = False
            dopsReceived    = False
            swiftSbpMsg = SwiftSbp()
            try:
                print "\n"
                for msg, metadata in source.filter():
                    #MSG POS ECEF
                    if msg.msg_type == 0x0200:
                        print "MSG POS ECEF 0x0200"
                        poseStampedECEF = PoseStamped()
                        poseStampedECEF.pose.position.x = msg.x
                        poseStampedECEF.pose.position.y = msg.y
                        poseStampedECEF.pose.position.z = msg.z                    
                    #MSG POS LLH
                    if msg.msg_type == 0x0201:
                        print "MSG POS LLH 0x0201"
                        # Flag [0-2]:
                        # 0 Single Point Positioning (SPP)
                        # 1 Fixed RTK
                        # 2 Float RTK
                        fixType = msg.flags & 0x07
                        # Only Send RTK Fixed Pose
                        if fixType == 1:
                            print 'Fixed RTK', fixType
                            swiftSbpMsg.flag                 = fixType
                            swiftSbpMsg.header.stamp         = rospy.Time.now()
                            swiftSbpMsg.latitude             = msg.lat * 10000000
                            swiftSbpMsg.longitude            = msg.lon * 10000000
                            swiftSbpMsg.height               = msg.height * 1000
                            swiftSbpMsg.tow                  = msg.tow
                            swiftSbpMsg.horizontal_accuracy  = msg.h_accuracy
                            swiftSbpMsg.vertical_accuracy    = msg.v_accuracy 
                            swiftSbpMsg.numOfSat             = msg.n_sats
                            poseLLHReceived = True
                    #MSG BASELINE NED
                    if msg.msg_type == 0x0203:
                        print "MSG BASELINE NED 0x0203"
                        dist = math.sqrt(msg.n*msg.n + msg.e*msg.e + msg.d*msg.d) 
                        swiftSbpMsg.baseline_north = msg.n 
                        swiftSbpMsg.baseline_east  = msg.e
                        swiftSbpMsg.baseline_down  = msg.d
                        swiftSbpMsg.distance       = dist
                        poseNEDReceived            = True
                    #MSG VEL NED
                    if msg.msg_type == 0x0205:    
                        print 'MSG VEL NED 0x0205'
                        flagMsg2 = True
                        swiftSbpMsg.vn = msg.n
                        swiftSbpMsg.ve = msg.e
                        swiftSbpMsg.vd = msg.d
                        velNEDReveived = True
                    #MSG DOPS - 0x0206 : comes at a slow rate for some reason
                    if msg.msg_type == 0x0206:
                        print 'MSG DOPS 0x0206'
                        swiftSbpMsg.hdop = msg.hdop
                        swiftSbpMsg.vdop = msg.vdop
                        dopsReceived     = True                        
                    #MSG BASELINE HEADING - 0x0207
                    if(poseLLHReceived and poseNEDReceived and velNEDReveived):
                        poseLLHReceived = False
                        poseNEDReceived = False
                        velNEDReveived  = False
                        dopsReceived    = False
                        pub.publish(swiftSbpMsg)                    
            except KeyboardInterrupt:
                pass
if __name__ == '__main__':
    try:
        gpsrtk()
    except rospy.ROSInterruptException:
        pass