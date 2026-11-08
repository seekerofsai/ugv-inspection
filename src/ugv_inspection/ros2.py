"""ROS 2 Humble adapter, intended to run inside the vendor ROS environment.

Uses NavigateToPose, Odometry, Image and the vendor's JointState COMMAND topic.
Hardware integration is untested. The command topic is not encoder feedback.
"""
import argparse
import json
import math
from pathlib import Path
import time

from .core import Config, Frame, Observation
from .cli import run


class RosBackend:
    def __init__(self, config, settings):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.node import Node
        from nav2_msgs.action import NavigateToPose
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Image, JointState
        from geometry_msgs.msg import Twist
        from cv_bridge import CvBridge
        import cv2
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError('OpenCV with aruco is required in the ROS Python environment')
        self.rclpy, self.cv2, self.cfg, self.s = rclpy, cv2, config, settings
        self.JointState, self.Twist, self.NavigateToPose = JointState, Twist, NavigateToPose
        rclpy.init()
        self.node = Node('ugv_inspection')
        self.origin = time.monotonic()
        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(self.dictionary) if hasattr(cv2.aruco,'ArucoDetector') else None
        self.pt = self.node.create_publisher(JointState,settings['pan_tilt_topic'],10)
        self.stop_pub = self.node.create_publisher(Twist,settings['stop_topic'],10)
        self.node.create_subscription(Image,settings['image_topic'],self.on_image,qos_profile_sensor_data)
        self.node.create_subscription(Odometry,settings['odom_topic'],self.on_odom,qos_profile_sensor_data)
        self.action = ActionClient(self.node,NavigateToPose,settings['navigate_action'])
        self.frame = None
        self.odom_at = None
        self.pose = (0.,0.,0.)
        self.speed = self.yaw_rate = float('inf')
        self.last_image_stamp = None
        self.target = config.targets[0]['id']
        self.nav = 'running'
        self.sent = self.result = self.handle = None
        self.halted = False
        self.cancel_future = None

    def now(self):
        return time.monotonic()-self.origin

    def source_age(self, header):
        stamp = header.stamp.sec+header.stamp.nanosec*1e-9
        age = self.node.get_clock().now().nanoseconds*1e-9-stamp
        # Unstamped or future-dated data cannot pass capture freshness checks.
        return age if stamp > 0 and age >= -0.05 else float('inf')

    def on_odom(self, msg):
        age = self.source_age(msg.header)
        self.odom_at = self.now()-max(0,age)
        v = msg.twist.twist
        self.speed, self.yaw_rate = math.hypot(v.linear.x,v.linear.y), v.angular.z
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.pose = (p.x,p.y,yaw)

    def on_image(self, msg):
        key = (msg.header.stamp.sec,msg.header.stamp.nanosec)
        age = self.source_age(msg.header)
        if key == self.last_image_stamp or age > self.cfg.frame_max_age:
            return
        self.last_image_stamp = key
        cv2 = self.cv2
        bgr = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        gray = cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
        corners, ids, _ = (self.detector.detectMarkers(gray) if self.detector else
                           cv2.aruco.detectMarkers(gray,self.dictionary))
        boxes, sharpness = {}, 0.0
        height,width = gray.shape
        if ids is not None:
            for marker, tag in zip(corners,ids.flatten()):
                pts = marker.reshape(4,2)
                x0,y0 = pts.min(axis=0)
                x1,y1 = pts.max(axis=0)
                boxes[int(tag)] = [float((x0+x1)/width-1), float((y0+y1)/height-1),
                                   float((x1-x0)/width), float((y1-y0)/height)]
                if int(tag) == self.target:
                    roi = gray[max(0,int(y0)):min(height,int(y1)+1),max(0,int(x0)):min(width,int(x1)+1)]
                    sharpness = min(1.0,float(cv2.Laplacian(roi,cv2.CV_64F).var())/self.s['laplacian_reference']) if roi.size else 0.0
        ok,png = cv2.imencode('.png',bgr)
        if not ok:
            raise RuntimeError('Camera PNG encoding failed')
        self.frame = Frame(self.now()-max(0,age),boxes,sharpness,png.tobytes())

    def start_navigation(self, goal):
        self.goal = goal

    def tick(self, dt, target_id):
        self.target = target_id
        self.rclpy.spin_once(self.node,timeout_sec=min(dt,0.02))
        now = self.now()
        odom_age = now-self.odom_at if self.odom_at is not None else float('inf')
        if not self.halted and self.sent is None:
            # Do not send a goal until sensors and the Nav2 action server are ready.
            if (self.action.server_is_ready() and odom_age <= self.cfg.frame_max_age and
                self.frame is not None and now-self.frame.stamp <= self.cfg.frame_max_age):
                goal = self.NavigateToPose.Goal()
                goal.pose.header.frame_id = self.s['map_frame']
                goal.pose.header.stamp = self.node.get_clock().now().to_msg()
                goal.pose.pose.position.x,goal.pose.pose.position.y = map(float,self.goal[:2])
                goal.pose.pose.orientation.z = math.sin(self.goal[2]/2)
                goal.pose.pose.orientation.w = math.cos(self.goal[2]/2)
                self.sent = self.action.send_goal_async(goal)
        if self.sent is not None and self.sent.done() and self.handle is None:
            self.handle = self.sent.result()
            if not self.handle.accepted:
                self.nav = 'failed'
            else:
                self.result = self.handle.get_result_async()
                if self.halted:
                    self.cancel_future = self.handle.cancel_goal_async()
        if self.result is not None and self.result.done():
            from action_msgs.msg import GoalStatus
            self.nav = 'succeeded' if self.result.result().status == GoalStatus.STATUS_SUCCEEDED else 'failed'
        if self.halted:
            self.stop_pub.publish(self.Twist())
        return Observation(now,self.nav,self.speed,self.yaw_rate,odom_age,self.frame,self.pose)

    def point(self,pan,tilt):
        msg = self.JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = [self.s['pan_joint'],self.s['tilt_joint']]
        msg.position = [self.s['pan_zero']+self.s['pan_sign']*pan,
                        self.s['tilt_zero']+self.s['tilt_sign']*tilt]
        self.pt.publish(msg)

    def stop(self):
        self.halted = True
        if (self.handle is not None and self.handle.accepted and self.result is not None
                and not self.result.done() and self.cancel_future is None):
            self.cancel_future = self.handle.cancel_goal_async()
        self.stop_pub.publish(self.Twist())

    def close(self):
        self.stop()
        # Service pending goal acceptance/cancellation and repeat zero velocity briefly.
        deadline = time.monotonic()+3.0
        while time.monotonic()<deadline and self.rclpy.ok():
            self.tick(0.02,self.target)
        if self.sent is not None and (self.handle is None or
            (self.handle.accepted and self.result is not None and not self.result.done())):
            self.node.get_logger().error('Navigation termination not confirmed; use the local stop control.')
        self.node.destroy_node()
        self.rclpy.shutdown()


def main():
    p = argparse.ArgumentParser(description='Run on commissioned rover in its sourced ROS 2 environment')
    p.add_argument('--mission',type=Path,required=True)
    p.add_argument('--hardware-config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--execute',action='store_true',help='Enable physical navigation and pan–tilt commands')
    args = p.parse_args()
    c = Config.load(args.mission)
    s = json.loads(args.hardware_config.read_text())
    if not args.execute or s.get('commissioned') is not True:
        p.error('Hardware requires --execute and commissioned=true after completing docs/HARDWARE.md')
    for name in ('pan_sign','tilt_sign'):
        if s[name] not in (-1,1):
            p.error(f'{name} must be -1 or 1')
    if not math.isfinite(s['laplacian_reference']) or s['laplacian_reference'] <= 0:
        p.error('laplacian_reference must be positive')
    if args.output.exists():
        p.error('Output must be a new directory')
    backend = RosBackend(c,s)
    report = run(c,args.output,scenario='hardware',backend=backend,realtime=True)
    (args.output/'hardware-config.json').write_text(json.dumps(s,indent=2)+'\n')
    print(f"{report['state']}: {args.output/'replay.html'}")
    return 0 if report['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
