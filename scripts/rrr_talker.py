#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from rrr_hexagon_subham import rrr_publisher
def talker():
    #create a new publisher. we specify the topic name, then type of message
    pub1 = rospy.Publisher('/rrr/joint1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/rrr/joint2_position_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/rrr/joint3_position_controller/command', Float64, queue_size=10)




    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    msg1 = Float64()
    msg2 = Float64()
    msg3 = Float64()
    xx=0
    while not rospy.is_shutdown():

        msg1.data = xx
        msg2.data = 0
        msg3.data = 0


        pub1.publish(msg1)
        pub2.publish(msg2)
        pub3.publish(msg3)
        xx = xx+0.01
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
        # rrr_publisher(0,0,1)
    except rospy.ROSInterruptException:
        pass