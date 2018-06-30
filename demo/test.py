import dynamic_reconfigure._dynamic_reconfigure_ as dy
import rclpy
from testRclConfig import testRclConfig_list

def callback(values):
    print("callback in python def callback():")
    print(values)

def main(args=None):
    rclpy.init(args=args)
    dy.params_service_init("A_service_name", callback, testRclConfig_list().data)
    rclpy.shutdown()

if __name__ == '__main__':
    main()