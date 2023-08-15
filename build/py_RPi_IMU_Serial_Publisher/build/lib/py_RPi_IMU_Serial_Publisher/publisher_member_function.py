import rclpy
import serial
from rclpy.node import Node

from geometry_msgs.msg import Vector3

class MinimalPublisher(Node):

    def __init__(self):
        
        #Serial settings:
        BAUD = 115200
        SERIAL_PORT = '/dev/pts/2'
        
        #Init serial object
        self.ser = serial.Serial(port = SERIAL_PORT, 
                    baudrate=BAUD,
                    timeout = None #Wait indefinitely for data.
                    )
        
        
        self.get_logger().info(f"Initializing Serial Port @ baud {BAUD} and port {SERIAL_PORT}")
        
        #Create node with name
        super().__init__('rpi_imu_ypr_publisher')

        #on the node, create publisher that sends a vector with a buffer queue of 10.
        self.publisher_ = self.create_publisher(Vector3, 'rpi_imu_ypr_topic', 10)

        #Define period and frequency
        timer_freq = 200 #Hz
        timer_period = 1/timer_freq  # seconds

        #Log Details
        self.get_logger().info(f"Publisher Created. Publishing to rpi_imu_ypr_topic at {timer_period} hz...")
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def parse_attitude_data(self, data):
        data = data[:-3].strip() #I have zero clue why it's -5. I think the ascii output of the sensor maybe puts whitespaces after the message? It should be -3 to accocunt for the lat 3 characters.
        values = data.split(",")

        if len(values) >= 4 and values[0] == "$VNYPR":
            yaw_att = float(values[1].strip())
            pitch_att = float(values[2].strip())
            roll_att  = float(values[3].strip())
            return yaw_att, pitch_att, roll_att
        
        return None
    
    def timer_callback(self):
        #Define message packet as the Vector3 message. Details: http://docs.ros.org/en/api/geometry_msgs/html/msg/Wrench.html
        msg = Vector3()
        
        #Read the top of the stack from the serial port
        attitude_data = self.ser.readline()
        
        #Parse attitude data and assign to message packet
        msg.x, msg.y, msg.z = self.parse_attitude_data(attitude_data)
        
        #Send packet to the topic
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: yaw: {msg.x}, pitch: {msg.y}, roll: {msg.z}')


def main(args=None):
    
    rclpy.init(args=args)
    
    #Create minimal publiser object
    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()