# zoro_dynamic_reconfigure

#### 1.编写cfg文件 

+ cfg文件写法与ros1一致

  group相关功能未实现，所有参数均属于root

http://wiki.ros.org/dynamic_reconfigure/Tutorials/HowToWriteYourFirstCfgFile

+ 编写完成后 可直接python运行cfg文件生成 ($name)RclConfig.h 和 ($name)RclConfig.py

```
python3 Tutorials.cfg
```

​	注意确保调用的parameter_generator_catkin.py是前路径里的，现在是调用当前目录下的parameter_generator_catkin.py。

```python
from .parameter_generator_catkin import *
```


#### 2.c++ server demo

```C++
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "testRclConfig.h" // created by *.cfg
#include "dynamic_reconfigure_server_cpp.hpp"
#include <vector>
#include <string>

void cb(std::map<std::string, boost::any> all_value) {  // call_back 
  printf("user callback\n");
  /*methods to get parameters of all types
  std::string str_param = boost::any_cast<std::string>(all_value["str_param"]);
  int64_t int_param = boost::any_cast<int64_t>(all_value["int_param"]);
  double double_param = boost::any_cast<double>(all_value["double_param"]);
  bool bool_param = boost::any_cast<bool>(all_value["bool_param"]);
  */
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("get_parameters3");
  auto ser_ptr = std::make_shared<rqt_reconfigure::Server<ConfigureVec>>(node, cb);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```



#### 3.python server demo

```python
import dynamic_reconfigure._dynamic_reconfigure_ as dy
import rclpy
from testRclConfig import testRclConfig_list # created by *.cfg

def callback(values):
    print(values) #values is a python dict 

def main(args=None):
    rclpy.init(args=args)
    dy.params_service_init("A_service_name", callback, testRclConfig_list().data)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
