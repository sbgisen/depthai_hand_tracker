#!/usr/bin/env python3
# -*- encoding: utf-8 -*-

import rclpy
from cv_bridge import CvBridge
# from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from depthai_hand_tracker.HandTrackerEdge import HandTracker
from depthai_hand_tracker.HandTrackerRenderer import HandTrackerRenderer


class HandTrackingNode(Node):
    def __init__(self) -> None:
        super().__init__('hand_tracking')

        self.tracker = HandTracker(
            use_gesture=True,
            xyz=False,
            stats=True,
            # **tracker_args
        )

        self.renderer = HandTrackerRenderer(
            tracker=self.tracker,
        )

        self.gesture_sign_pub = self.create_publisher(String, 'hand_gesture', 10)
        self.image_pub = self.create_publisher(Image, 'hand_detected_image', 10)
        # self.palm_point_pub = self.create_publisher(Point, 'palm_point', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10hz
        self.bridge = CvBridge()

    def timer_callback(self) -> None:
        try:
            frame, hands, bag = self.tracker.next_frame()
            if frame is not None:
                frame = self.renderer.draw(frame, hands, bag)
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            if len(hands) > 0:
                gesture_sign = hands[0].gesture
                self.get_logger().info(str(gesture_sign))
                # palm_point = Point(x=hands[0].xyz[0], y=hands[0].xyz[1], z=hands[0].xyz[2])
                msg = String()
                msg.data = str(gesture_sign)
                self.gesture_sign_pub.publish(msg)
                # self.palm_point_pub.publish(palm_point)
        except RuntimeError as e:
            self.get_logger().error(f"RuntimeError: {str(e)}")

    def destroy(self) -> None:
        self.renderer.exit()
        self.tracker.exit()
        super().destroy()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandTrackingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
