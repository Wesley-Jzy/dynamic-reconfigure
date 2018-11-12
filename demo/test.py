from dynamic_reconfigure.server import Server
import rclpy
from testRclConfig import testRclConfig_list
import time

def callback(values):
    print("callback in python def callback():")
    print(values)

def main(args=None):
    rclpy.init(args=args)
    Server("A_service_name", callback, testRclConfig_list().data)
    while rclpy.ok():#what you want to do
        print("haha")
        time.sleep(2) 
    

if __name__ == '__main__':
    main()
