import rclpy
from rclpy.node import Node
import socket

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        #Create node with name
        super().__init__('rpi_imu_ypr_publisher')

        #on the node, create publisher that sends a string with a buffer queue of 10.
        self.publisher_ = self.create_publisher(String, 'rpi_imu_ypr_topic', 10)

        #Define period and frequency
        timer_hz = 200
        timer_freq = 1/timer_hz  # seconds

        #Log Details
        self.get_logger().info(f"Publisher Created. Publishing to rpi_imu_ypr_topic at {timer_hz} hz...")

        #Define and Assign TCP socket
        tcp_receiver_socket = self.tcp_init()

        #Accept connection from sender
        tcp_sender_socket = self.socket_connect(tcp_receiver_socket)

        #assign callback function to node
        self.timer = self.create_timer(timer_freq, lambda: self.timer_callback(tcp_sender_socket, tcp_receiver_socket))

        


    def timer_callback(self, sender_socket, receiver_socket):
        #Define message type
        msg = String()

        #Check if sender_socket is not empty
        if sender_socket:

            # Flush unread data in the buffer
            sender_socket.setblocking(0)
            while True:
                try:
                    sender_socket.recv(1024)
                except BlockingIOError:
                    break
            sender_socket.setblocking(1)

            # Receive data from the sender
            msg.data = sender_socket.recv(1024).decode()
            msg.data = msg.data

            #If msg.data is non empty
            if msg.data:
                self.publisher_.publish(msg)
                self.get_logger().info('Publishing YPR Data: "%s"' % msg.data)
            else:
                self.get_logger().error("Error: No data received. Checking if socket is still receiving data.")

                #Check if socket is still active
                if not self.check_socket_activity(receiver_socket):

                    self.get_logger().info("Socket is no longer active.")

                    #for i in range(5,0,-1):
                    #    print(f"Retrying connection in {i} seconds...")
                    #    time.sleep(1)


    def tcp_init(self):
        #receiver_ip = "192.168.2.2" # NUC IP if on Belkin Router --> Check router settings for this IP.
        receiver_ip = "169.254.48.36" #NUC IP if on LabSwitch Eth Connection. --> Check wired internet settings for this IP.

        receiver_port = 8888 # User Defined.
        receiver_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #TCP Socket Initialization
        
        # Bind the socket to a specific IP and port
        try:
            receiver_socket.bind((receiver_ip, receiver_port))
            self.get_logger().info(f"Socket successfully bound to address: ({receiver_ip}, {receiver_port})")
            return receiver_socket
        
        #If get a network connection error.
        except OSError as e:
            self.get_logger().fatal(e)
            self.get_logger().fatal("OSError occurred. Please check your network settings. Exiting Script.")
            exit()


    # Function to connect to the socket and accept incoming connections
    def socket_connect(self, receiver_sock):
        sender_socket = None
        try:
            # Listen for incoming connections
            receiver_sock.listen(1)
            self.get_logger().info("Listening for incoming connections.")

            # Accept a connection from the sender
            sender_socket, sender_address = receiver_sock.accept()
            self.get_logger().info(f"Connection accepted from {sender_address}.")
            
        except socket.timeout:
            self.get_logger().error("Connection timed out.")
            
        #Return the socket that is sending data.
        return sender_socket

    # Function to check socket activity
    def check_socket_activity(self, sock):
        try:
            sock.settimeout(5)
            # Attempt to receive data from the socket
            data = sock.recv(1024).decode()
            if data:
                return True  # Data received
            else:
                return False  # No data received
        except socket.timeout:
            return False  # Socket timed out


def main(args=None):

    # Initialize ROS node
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
