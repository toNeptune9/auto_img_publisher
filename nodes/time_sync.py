#!/usr/bin/env python
import message_filters
import rospy
from sensor_msgs.msg import Image, CameraInfo

def callback(image, camera_info):
  im_p.publish(image);
  in_p.publish(camera_info)

im_p = rospy.Publisher('image_new', Image, queue_size=10)
in_p = rospy.Publisher('camera_info_new', CameraInfo, queue_size=10)       

if __name__ == '__main__':
    try:
        rospy.init_node('timesync', anonymous=False)
        image_sub = message_filters.Subscriber('/ws/img_bucket/image_raw', Image)
        info_sub = message_filters.Subscriber('/ws/img_bucket/camera_info', CameraInfo)
        ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
        ts.registerCallback(callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
