import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import pickle
import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist

from shapely.geometry import Point, Polygon

package_name = 'gnss_safety_checker'  
pkg_share = get_package_share_directory(package_name)
config_converted_outer_path = os.path.join(pkg_share, 'config', 'converted_outer.pkl')
config_converted_inner_path = os.path.join(pkg_share, 'config', 'converted_inner.pkl')

with open(config_converted_inner_path, "rb") as f:
    forbidden_inside_zone_coords = pickle.load(f)

with open(config_converted_outer_path, "rb") as f:
    forbidden_outside_zone_coords = pickle.load(f)

forbidden_inside_zone = Polygon(forbidden_inside_zone_coords)
forbidden_outside_zone = Polygon(forbidden_outside_zone_coords)

class GnssTest(Node):
    def __init__(self):
        super().__init__('gnss_test')
        buffer_size = 10
        self.gnss_sub_ = self.create_subscription(NavSatFix, 'gnss', self.gnss_callback, buffer_size)
        self.coasting_twist_pub = self.create_publisher(Twist, 'pub_cmd_vel_gnss', buffer_size)
        

    def gnss_callback(self,msg):
        print(f"latitude:{msg.latitude}")
        print(f"longitude:{msg.longitude}")
        current_position = Point(msg.latitude,msg.longitude)

        if forbidden_inside_zone.contains(current_position) or not forbidden_outside_zone.contains(current_position):
            print("out!!")
        else:
            print("ok")

def main(args=None):
    rclpy.init(args=args)
    gnss_test = GnssTest()
    rclpy.spin(gnss_test)
    gnss_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
