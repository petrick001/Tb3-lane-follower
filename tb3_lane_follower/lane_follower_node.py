#!/usr/bin/env python3
"""ROS2 node: camera image -> lane error -> /cmd_vel   (runs on the remote PC)."""
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

from tb3_lane_follower.controller import ControlParams, PDController
from tb3_lane_follower.lane_detector import DetectorParams, detect


def image_to_bgr(msg: Image):
    """sensor_msgs/Image -> BGR numpy array (no cv_bridge needed)."""
    buf = np.frombuffer(msg.data, np.uint8)
    if msg.encoding in ('bgr8', 'rgb8'):
        arr = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return arr if msg.encoding == 'bgr8' else cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    if msg.encoding == 'mono8':
        arr = buf.reshape(msg.height, msg.step)[:, :msg.width]
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    return None


class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')
        d, c = DetectorParams(), ControlParams()

        # ---- node parameters -------------------------------------------------
        self.declare_parameter('image_topic', '/camera/image_raw/compressed')
        self.declare_parameter('use_compressed', True)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('enabled', True)               # False = detect only, never move
        self.declare_parameter('show_window', False)          # cv2 window on the PC screen
        self.declare_parameter('image_timeout', 0.6)          # s without image -> stop
        self.declare_parameter('lost_hold_frames', 5)         # keep last command this many frames
        self.declare_parameter('lost_recover_w', 0.0)         # rad/s turn toward last side when lost (0 = just stop)
        # detector
        self.declare_parameter('proc_width', d.proc_width)
        self.declare_parameter('yellow_h', list(d.yellow_h))
        self.declare_parameter('yellow_s_min', d.yellow_s_min)
        self.declare_parameter('yellow_v_min', d.yellow_v_min)
        self.declare_parameter('white_s_max', d.white_s_max)
        self.declare_parameter('white_v_min', d.white_v_min)
        self.declare_parameter('white_abs_v_min', d.white_abs_v_min)
        self.declare_parameter('min_contrast', d.min_contrast)
        self.declare_parameter('blob_size', d.blob_size)
        self.declare_parameter('roi_top', d.roi_top)
        self.declare_parameter('band_top', d.band_top)
        self.declare_parameter('band_bottom', d.band_bottom)
        self.declare_parameter('min_pixels', d.min_pixels)
        self.declare_parameter('half_lane', d.half_lane)
        self.declare_parameter('yellow_on_left', d.yellow_on_left)
        # controller
        for k in ('kp', 'kd', 'v_max', 'v_min', 'w_max', 'slowdown', 'smooth'):
            self.declare_parameter(k, getattr(c, k))

        self.det = self._build_detector_params()
        self.ctrl = PDController(self._build_control_params())

        # ---- I/O ------------------------------------------------------------
        topic = self.get_parameter('image_topic').value
        if self.get_parameter('use_compressed').value:
            self.create_subscription(CompressedImage, topic, self.on_compressed, qos_profile_sensor_data)
        else:
            self.create_subscription(Image, topic, self.on_raw, qos_profile_sensor_data)
        self.pub_cmd = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.pub_dbg = self.create_publisher(Image, '~/debug_image', 1)
        self.create_timer(0.1, self.watchdog)

        self._dirty = False
        self.last_img_t = None
        self.last_t = None
        self.lost = 0
        self.last_cmd = (0.0, 0.0)
        self.last_side = 0.0
        self.add_on_set_parameters_callback(self._on_params)
        self.get_logger().info('lane_follower started: topic=%s enabled=%s' %
                               (topic, self.get_parameter('enabled').value))

    # ---- parameters ---------------------------------------------------------
    def _build_detector_params(self):
        g = lambda n: self.get_parameter(n).value
        yh = g('yellow_h')
        return DetectorParams(
            proc_width=int(g('proc_width')), yellow_h=(int(yh[0]), int(yh[1])),
            yellow_s_min=int(g('yellow_s_min')), yellow_v_min=int(g('yellow_v_min')),
            white_s_max=int(g('white_s_max')), white_v_min=int(g('white_v_min')),
            white_abs_v_min=int(g('white_abs_v_min')), min_contrast=int(g('min_contrast')),
            blob_size=int(g('blob_size')), roi_top=float(g('roi_top')),
            band_top=float(g('band_top')), band_bottom=float(g('band_bottom')),
            min_pixels=int(g('min_pixels')), half_lane=float(g('half_lane')),
            yellow_on_left=bool(g('yellow_on_left')))

    def _build_control_params(self):
        g = lambda n: float(self.get_parameter(n).value)
        return ControlParams(kp=g('kp'), kd=g('kd'), v_max=g('v_max'), v_min=g('v_min'),
                             w_max=g('w_max'), slowdown=g('slowdown'), smooth=g('smooth'))

    def _on_params(self, params):
        # live tuning:  ros2 param set /lane_follower kp 1.5
        # (values are applied right after this callback, so rebuild on the next frame)
        from rcl_interfaces.msg import SetParametersResult
        self._dirty = True
        return SetParametersResult(successful=True)

    def _refresh(self):
        self.det = self._build_detector_params()
        self.ctrl.p = self._build_control_params()

    # ---- image callbacks ------------------------------------------------------
    def on_compressed(self, msg: CompressedImage):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            self.process(img)

    def on_raw(self, msg: Image):
        img = image_to_bgr(msg)
        if img is None:
            self.get_logger().warn('unsupported encoding: %s' % msg.encoding, throttle_duration_sec=5.0)
            return
        self.process(img)

    # ---- main logic -------------------------------------------------------------
    def process(self, bgr):
        if self._dirty:          # params changed live -> rebuild (after they are applied)
            self._refresh()
            self._dirty = False
        now = time.monotonic()
        dt = 0.0 if self.last_t is None else now - self.last_t
        self.last_t, self.last_img_t = now, now

        res = detect(bgr, self.det, draw=True)

        if res.error is not None:
            self.lost = 0
            self.last_side = 1.0 if res.error > 0 else -1.0
            v, w = self.ctrl.update(res.error, dt)
            self.last_cmd = (v, w)
        else:
            self.lost += 1
            if self.lost <= int(self.get_parameter('lost_hold_frames').value):
                v, w = self.last_cmd                    # coast on the last command briefly
            else:
                rw = float(self.get_parameter('lost_recover_w').value)
                v, w = (0.0, -self.last_side * rw)      # stop (optionally turn toward last side)
                self.ctrl.reset()

        self.send(v, w)
        self.publish_debug(res.debug, v, w)

    def send(self, v, w):
        msg = Twist()
        if self.get_parameter('enabled').value:
            msg.linear.x, msg.angular.z = float(v), float(w)
        self.pub_cmd.publish(msg)

    def publish_debug(self, dbg, v, w):
        if dbg is None:
            return
        cv2.putText(dbg, 'v=%.2f w=%+.2f' % (v, w), (5, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        if self.get_parameter('show_window').value:
            cv2.imshow('lane_follower', dbg)
            cv2.waitKey(1)
        if self.pub_dbg.get_subscription_count() > 0:
            m = Image()
            m.header.stamp = self.get_clock().now().to_msg()
            m.height, m.width = dbg.shape[:2]
            m.encoding, m.step = 'bgr8', dbg.shape[1] * 3
            m.data = dbg.tobytes()
            self.pub_dbg.publish(m)

    def watchdog(self):
        """No images for a while -> stop the robot."""
        t = self.last_img_t
        if t is not None and time.monotonic() - t > float(self.get_parameter('image_timeout').value):
            self.pub_cmd.publish(Twist())
            self.ctrl.reset()

    def stop(self):
        try:
            self.pub_cmd.publish(Twist())
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()                      # always leave the robot stopped
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
