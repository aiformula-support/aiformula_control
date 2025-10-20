import rclpy
from typing import List
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import pickle
import os
from ament_index_python.packages import get_package_share_directory
from can_msgs.msg import Frame
from geometry_msgs.msg import Twist
from enum import IntEnum

from shapely.geometry import Point, Polygon

from common_python.get_ros_parameter import get_ros_parameter

# package_name = 'road_depature_preventer'
# pkg_share = get_package_share_directory(package_name)
# config_converted_outer_path = os.path.join(pkg_share, 'config', 'converted_outer.pkl')
# config_converted_inner_path = os.path.join(pkg_share, 'config', 'converted_inner.pkl')

# with open(config_converted_inner_path, "rb") as f:
#     forbidden_inside_zone_coords = pickle.load(f)

# with open(config_converted_outer_path, "rb") as f:
#     forbidden_outside_zone_coords = pickle.load(f)

# forbidden_inside_zone = Polygon(forbidden_inside_zone_coords)
# forbidden_outside_zone = Polygon(forbidden_outside_zone_coords)


class DriveWheel(IntEnum):
    LEFT = 0
    RIGHT = 1
    NUM_DRIVE_WHEELS = 2

class ControlMode(IntEnum):
    ESTOP = 0
    GNSS_STOP = 1
    JOY_OVER_RIDE= 2

class PublishMode(IntEnum):
    WAIT = 0
    READY = 1
    ALREADY = 2

class GnssTest(Node):
    
    EMERGENCY_CAN_ID = 1808
    EMERGENCY_CAN_DLC = 4
    
    def __init__(self):
        super().__init__('gnss_test')
        self.get_ros_params()
        
        buffer_size = 10
        self.gnss_sub_ = self.create_subscription(NavSatFix, 'gnss', self.gnss_callback, buffer_size)
        # self.emergency_stop_can_sub_ = self.create_subscription()
        self.can_pub = self.create_publisher(Frame, 'pub_can', buffer_size)
        self.twist_pub = self.create_publisher(Twist, 'pub_twist', buffer_size)
        self.publish_timer = self.create_timer(self.publish_timer_loop_duration, self.publish_canframe_callback)
        # CAN Frame Set
        self.frame_msg = Frame()
        self.flag_msg = Frame()
        self.frame_msg.header.frame_id = "can0"        # Default can0
        self.frame_msg.id = 0x291                      # MotorController CAN ID : 0x291
        self.frame_msg.dlc = 8                         # Data length
        self.flag_msg.header.frame_id = "can0"     # Default can0
        self.flag_msg.id = 0x292                   # FLAG MODE CAN ID : 0x292
        self.flag_msg.dlc = 8                      # Data length
        
 
        self.is_publish_can_msgs = False
        self.publish_estop_msgs_mode = PublishMode.WAIT
        self.publish_estop_msgs_counter = 0
        self.is_push_emergency_stop = False
        self.is_over_area = False
        self.LEFT_WHEEL_COMMAND_RPM = 0
        self.RIGHT_WHEEL_COMMAND_RPM = 0
        self.STOP_TWIST_MSG = Twist()
        
        self.load_boudary()

        
    def load_boudary(self):
        package_name = 'road_depature_preventer'
        pkg_share = get_package_share_directory(package_name)
        config_converted_outer_path = os.path.join(pkg_share, 'config', 'converted_outer.pkl')
        config_converted_inner_path = os.path.join(pkg_share, 'config', 'converted_inner.pkl')

        with open(config_converted_inner_path, "rb") as f:
            forbidden_inside_zone_coords = pickle.load(f)

        with open(config_converted_outer_path, "rb") as f:
            forbidden_outside_zone_coords = pickle.load(f)

        self.forbidden_inside_zone = Polygon(forbidden_inside_zone_coords)
        self.forbidden_outside_zone = Polygon(forbidden_outside_zone_coords)        

    def get_ros_params(self):
        self.diameter = get_ros_parameter(self, "wheel.diameter")
        self.tread = get_ros_parameter(self, "wheel.tread")
        self.gear_ratio = get_ros_parameter(self, "wheel.gear_ratio")
        self.publish_timer_loop_duration = get_ros_parameter(self, "publish_timer_loop_duration")
        

    def initialize_can_command(self):
        cmd_left = self.toCanCmd(self.LEFT_WHEEL_COMMAND_RPM)
        cmd_right = self.toCanCmd(self.RIGHT_WHEEL_COMMAND_RPM)
        can_data = cmd_right + cmd_left
        self.frame_msg.data = can_data
        self.flag_msg.data[0] = 1

# -----------------------------  ESTOP relese ----------------------------------------
    def can_frame_callback(self, msg: Frame):
        self.get_logger().info_once("Subscribe Can Frame !")
        # if msg.id != EMERGENCY_CAN_ID:
        #     return

        # if msg.dlc != EMERGENCY_CAN_DLC:
        #     self.get_logger().warn(f"Invalid DLC: expected 4, got {msg.dlc} (id=0x{msg.id:X})")
        #     return

        # if msg.data[0] == 8 and msg.data[2] == 8:
        #     if not self.is_push_emergency_stop:
        #         self.is_push_emergency_stop = True
        # elif self.is_push_emergency_stop:
        #     self.is_push_emergency_stop = False
        

    def gnss_callback(self,msg):
        current_position = Point(msg.latitude, msg.longitude)
        # check area
        if self.forbidden_inside_zone.contains(current_position) or not self.forbidden_outside_zone.contains(current_position):
            self.is_over_area = True
        else:
            self.is_over_area = False
            
        # check publish status
        if self.publish_estop_msgs_mode == PublishMode.WAIT and self.is_over_area:
            self.publish_estop_msgs_mode = PublishMode.READY
            
        if self.publish_estop_msgs_mode == PublishMode.READY and self.publish_estop_msgs_counter > 100:
            self.publish_estop_msgs_mode = PublishMode.ALREADY
            self.publish_estop_msgs_counter = 0
            
        if self.publish_estop_msgs_mode == PublishMode.ALREADY and self.is_over_area == False:
            self.publish_estop_msgs_mode = PublishMode.WAIT
            
        if self.publish_estop_msgs_counter <= 100 and self.publish_estop_msgs_mode == PublishMode.READY:
            self.publish_estop_msgs_counter += 1
        
        if self.is_publish_can_msgs and self.is_push_emergency_stop:
            self.is_publish_can_msgs = False
            
    def publish_canframe_callback(self):
        if self.publish_estop_msgs_mode == PublishMode.READY:
            self.can_pub.publish(self.frame_msg)
            self.can_pub.publish(self.flag_msg)
            self.twist_pub.publish(self.STOP_TWIST_MSG)
            
    #  Velocity -> RPM Calc
    #  V_right = (V + tread/2 * w)   [m/s]
    #  V_left = (V - tread / 2 * w ) [m/s]
    #  w_right = V/r + (d/2r) * w    [rad/s]
    #  w_left = V/r - (d/2r) * w     [rad/s]
    #  rpm = w * 60 / 2* pi          [rpm]
    #  rpm = rpm * gear_ratio        [rpm]

    def toRefRPM(self, linear_velocity, angular_velocity):  # Calc Motor ref rad/s
        wheel_angular_velocities = np.zeros(DriveWheel.NUM_DRIVE_WHEELS)
        wheel_angular_velocities[DriveWheel.LEFT] = (
            linear_velocity / (self.diameter * 0.5)) - (self.tread / self.diameter) * angular_velocity  # [rad/s]
        wheel_angular_velocities[DriveWheel.RIGHT] = (
            linear_velocity / (self.diameter * 0.5)) + (self.tread / self.diameter) * angular_velocity  # [rad/s]
        minute_to_second = 60.
        rpm = wheel_angular_velocities * (minute_to_second / (2. * np.pi))
        if rpm[DriveWheel.LEFT] * rpm[DriveWheel.RIGHT] < 0.0:
            rpm[:] = 0.0
            self.get_logger().debug(f"Preventing in-situ rotation ! (rpm: {rpm})")
        return (rpm * self.gear_ratio).tolist()

    @staticmethod
    def toCanCmd(rpm: float) -> List[int]:
        rounded = round(rpm)
        bytes = rounded.to_bytes(4, "little", signed=True)
        return list(bytes)


def main(args=None):
    rclpy.init(args=args)
    gnss_test = GnssTest()
    rclpy.spin(gnss_test)
    gnss_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
