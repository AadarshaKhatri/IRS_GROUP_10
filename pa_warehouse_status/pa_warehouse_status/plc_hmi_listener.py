import rclpy 
from rclpy.node import Node
from std_msgs.msg import String
import json 


class PlcHmiListener(Node):

    def __init__(self):
        super().__init__("plc_hmi_listener")
        self.subscription = self.create_subscription(String,"/hmi/unified_status",self.listener_callback,10)


    def listener_callback(self, msg):

        try:
            data = json.loads(msg.data) # parse JSON string into Python dict
            stamp = data["stamp"]
            box = data["box"]
            counts = data["counts"]
            print("📥📥 Received PLC status:")
            print(f" ⏱ Time: {stamp['sec']}.{stamp['nanosec']}")
            print(f" 📦📦 Box weight raw={box['weight_raw']}")
            print(f" 📍📍 Location: {box['location']}")
            print(f" 🔢🔢 Counts: big={counts['big']}, medium={counts['medium']}, "
            f"small={counts['small']}, total={counts['total']}")
            print() # ���� empty line at the end

        except Exception as e:
            self.get_logger().error(f"Failed to parse JSON: {e}\nRaw msg={msg.data}")


def main(args=None):
        rclpy.init(args=args)

        plc_hmi_listener = PlcHmiListener()

        rclpy.spin(plc_hmi_listener)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        plc_hmi_listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()