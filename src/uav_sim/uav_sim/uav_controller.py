import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Wrench
from ros_gz_interfaces.msg import EntityWrench

ENTITY_NONE      = 0
ENTITY_LIGHT     = 1
ENTITY_MODEL     = 2
ENTITY_LINK      = 3
ENTITY_VISUAL    = 4
ENTITY_COLLISION = 5
ENTITY_SENSOR    = 6
ENTITY_JOINT     = 7

class UAVController(Node):
    def __init__(self):
        super().__init__("uav_controller")
        
        self.sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.update_force,
            10
        )

        self.pub = self.create_publisher(
            EntityWrench,
            "/world/empty/wrench",
            10
        )

        self.timer = self.create_timer(
            0.001,
            self.pub_force,
        )

        self.current_force = 0.0
    
    def update_force(self, msg: Twist):
        if msg.linear.x > 0.0:
            self.current_force = 11.0
        else:
            self.current_force = 0.0
        self.get_logger().info('Updated to : "%d"' %self.current_force)

    def pub_force(self):
        entity_msg = EntityWrench()
        entity_msg.entity.name = "drone_box::base_link"
        entity_msg.entity.type = ENTITY_LINK 
        
        entity_msg.wrench.force.z = self.current_force

        self.pub.publish(entity_msg)

def main(args = None):
    rclpy.init(args=args)

    uav_controller = UAVController()

    try:
        rclpy.spin(uav_controller)
    except KeyboardInterrupt:
        pass
    finally:
        uav_controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
