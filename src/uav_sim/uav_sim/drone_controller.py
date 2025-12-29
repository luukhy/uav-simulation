import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Wrench

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)

        self.publisher_ = self.create_publisher(Wrench, '/drone/thrust', 10)
        
        self.get_logger().info('Drone Controller Started! Press "w" to fly.')

    def listener_callback(self, msg):
        wrench_msg = Wrench()
        
        if msg.linear.x > 0.0:
            wrench_msg.force.z = 100.0  
            self.get_logger().info('Thrusting UP!')
        else:
            wrench_msg.force.z = 0.0  
            
        self.publisher_.publish(wrench_msg)

def main(args = None):
    rclpy.init(args=args)

    drone_controller = DroneController()

    try:
        rclpy.spin(drone_controller)
    except KeyboardInterrupt:
        pass
    finally:
        drone_controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == 'main':
    main()
