#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

def calc_heading(time, latlon):
    """
    Calculate the heading to face polaris.

    Parameters
    ----------
    time : float
        UTC Timestamp.
    latlon : tuple
        (latitude, longitude)

    Returns
    -------
    Heading : float
        Heading in degrees from North.
    """
    
    # TODO: Find polaris and find yaw to it
    # Could just cheat and use a magnetometer. 
    # Doesn't work within ~1 degree of the poles but that's prob acceptable
    heading = 90
    
    return heading
    

def heading_polaris():
    pub = rospy.Publisher('heading_polaris', Float64, queue_size=10)
    
    # Get Lat/Lon subscriber
    # Just hardcoding for now
    
    rospy.init_node('heading_polaris', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        latlon = (0, 0) # hardcoded for testing, should add params or GPS
        
        heading = calc_heading(rospy.get_time(), latlon)
        
        rospy.loginfo("Current polaris heading is %s" % str(heading))
        pub.publish(heading)
        rate.sleep()

if __name__ == '__main__':
    try:
        heading_polaris()
    except rospy.ROSInterruptException:
        pass