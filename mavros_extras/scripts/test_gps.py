#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix

def callback(data):
    rospy.loginfo("Longitude: %f, Altitude %f" % (data.longitude, data.latitude))
    longitude = data.longitude
    latitude = data.latitude
    #Call for the programName(longitude,latitude)

def infoGetter():

    rospy.init_node('infoGetter', anonymous=True)

    rospy.Subscriber("sensor_msgs/NavSatFix", NavSatFix, callback)#First parameter is the name of the topic

    rospy.spin()

def GpsSender():
    
    pub = rospy.Publisher('/mavros/gps_reroute/gps_fix', NavSatFix, queue_size=10)    
    rospy.init_node('gps_hil_test', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg = NavSatFix()
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        msg.latitude  = 244480286
        msg.longitude = 543955448
        msg.altitude  = 10000
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        GpsSender()
    except rospy.ROSInterruptException:
        pass

