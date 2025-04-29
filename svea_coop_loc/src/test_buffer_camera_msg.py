#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo

class Republisher:
    def __init__(self):
        rospy.init_node('republisher_node', anonymous=True)

        self.original_topic = '/camera/image_raw'
        self.republish_topic = '/camera/image_raw/more'

        self.original_topic2 = '/camera/camera_info' 
        self.republish_topic2 = '/camera/camera_info/more' 

        self.rate = rospy.Rate(60)
        self.buffered_message = None
        self.buffered_message2 = None

        rospy.Subscriber(self.original_topic, Image, self.callback)
        rospy.Subscriber(self.original_topic2, CameraInfo, self.callback2)
        
        self.publisher = rospy.Publisher(self.republish_topic, Image, queue_size=1)
        self.publisher2 = rospy.Publisher(self.republish_topic2, CameraInfo, queue_size=1)
        self.published_buffered_msg = False
        self.published_buffered_msg2 = False

    def callback(self, msg):
        self.published_buffered_msg = False
        self.buffered_message = msg

    def callback2(self, msg):
        self.published_buffered_msg2 = False
        self.buffered_message2 = msg

    def republish(self):
        while not rospy.is_shutdown():
            if self.buffered_message is not None:
                if self.published_buffered_msg:
                    self.buffered_message.header.stamp += rospy.Duration.from_sec(1/60)
                    self.publisher.publish(self.buffered_message)
                else:
                    self.publisher.publish(self.buffered_message)
                    self.published_buffered_msg = True
            if self.buffered_message2 is not None:
                if self.published_buffered_msg2:
                    self.buffered_message2.header.stamp += rospy.Duration.from_sec(1/60)
                    self.publisher2.publish(self.buffered_message2)
                else:
                    self.publisher2.publish(self.buffered_message2)
                    self.published_buffered_msg2 = True
            self.rate.sleep()


if __name__ == '__main__':
    try:
        republisher = Republisher()
        republisher.republish()
    except rospy.ROSInterruptException:
        pass
