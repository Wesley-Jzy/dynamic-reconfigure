from dynamic_reconfigure.server import Server
import rclpy
from testRclConfig import testRclConfig_list

def callback(values):
    print("callback in python def callback():")
    print(values)

def main(args=None):
    rclpy.init(args=args)
    Server("A_service_name", callback, testRclConfig_list().data)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
