import rclpy
import serial
import time
from rclpy.node import Node

from geometry_msgs.msg import Vector3

# --------------- PARAMS --------------- #

#Serial port settings:
BAUD = 115200
SERIAL_PORT = '/dev/serial0'

#Topic details
TOPIC_PUBLISHED = 'rpi_imu_ypr_topic'
TOPIC_BUFFER_QUEUE = 10

#Define period and frequency
TIMER_FREQ = 200 #Hz
TIMER_PERIOD = 1/TIMER_FREQ  # seconds

class MinimalPublisher(Node):

    def __init__(self):
        
        #Create node with name
        super().__init__('rpi_imu_ypr_publisher')
        
        try:
            #Init serial object
            self.ser = serial.Serial(port = SERIAL_PORT, 
                        baudrate=BAUD,
                        timeout = None #Wait indefinitely for data.
                        )
        except serial.SerialException: #If issue with serial object instantiation
            self.get_logger().error(f"There was an issue with initializing the serial port object at port {SERIAL_PORT} @ {BAUD} baudrate. Check if the serial connection is running.")
            
            self.get_logger().info("Exiting.")
            exit()
        
        #Stabilize serial port
        self.get_logger().info(f"Initializing Serial Port @ baud {BAUD} and port {SERIAL_PORT}...\nWaiting for serial port to stabilize.")
        time.sleep(2)
        
        #on the node, create publisher that sends a vector with a buffer queue of 10.
        self.publisher_ = self.create_publisher(Vector3, TOPIC_PUBLISHED, TOPIC_BUFFER_QUEUE)
        self.get_logger().info(f"Created publisher to topic {TOPIC_PUBLISHED} with a buffer queue of {TOPIC_BUFFER_QUEUE}")

        #Log Details
        self.get_logger().info(f"Publisher Created. Publishing to rpi_imu_ypr_topic at {TIMER_PERIOD} hz...")
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

    def parse_attitude_data(self, data):
        #Decode/strip data and format
        data = data[:-5].decode().strip() # It's -5 because It's including the \n character. If without the \n, it's -3.
        
        #Split values with comma as delimiter
        values = data.split(",")
        
        #Format data and assign data
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
        parsed_data = self.parse_attitude_data(attitude_data)
        
        #Check if parsed_data is the right format or not
        if parsed_data:
            msg.x, msg.y, msg.z = parsed_data
            #send packet to topic
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: yaw: {msg.x}, pitch: {msg.y}, roll: {msg.z}')
        else: 
            self.get_logger().warning('Failed to parse attitude data. Check format.')


def main(args=None):
    
    #Initialize rclpy
    rclpy.init(args=args)
    
    #Create minimal publiser object
    minimal_publisher = MinimalPublisher()

    #Spin node
    rclpy.spin(minimal_publisher)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()