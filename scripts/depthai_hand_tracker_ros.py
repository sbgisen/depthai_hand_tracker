#!/usr/bin/env python3
# -*- encoding: utf-8 -*-


import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import String

from depthai_hand_tracker.HandTrackerEdge import HandTracker
from depthai_hand_tracker.HandTrackerRenderer import HandTrackerRenderer


def gesture() -> None:

    tracker = HandTracker(
        use_gesture=True,
        xyz=True,
        stats=True,
        # **tracker_args
    )

    renderer = HandTrackerRenderer(
        tracker=tracker,
    )

    gesture_sign_pub = rospy.Publisher('gesture', String, queue_size=10)
    image_pub = rospy.Publisher('detected_image', Image, queue_size=10)
    palm_point_pub = rospy.Publisher('palm_point', Point, queue_size=10)
    rospy.init_node('hand_tracking', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        # Run hand tracker on next frame
        # 'bag' contains some information related to the frame
        # and not related to a particular hand like body keypoints in Body Pre Focusing mode
        # Currently 'bag' contains meaningful information only when Body Pre Focusing is used
        # Draw hands
        frame, hands, bag = tracker.next_frame()
        if frame is not None:
            frame = renderer.draw(frame, hands, bag)
            image_pub.publish(CvBridge().cv2_to_imgmsg(frame, "bgr8"))
        if len(hands) > 0:
            gesture_sign = hands[0].gesture
            palm_point = Point(hands[0].xyz[0], hands[0].xyz[1], hands[0].xyz[2])
            gesture_sign_pub.publish(gesture_sign)
            palm_point_pub.publish(palm_point)
        rate.sleep()
    renderer.exit()
    tracker.exit()


if __name__ == '__main__':
    try:
        gesture()
    except rospy.ROSInterruptException:
        pass
