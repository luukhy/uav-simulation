import rclpy 
from rclpy.node import Node 
from std_msgs.msg import String 
from geometry_msgs.msg import Twist
from keyboard_msgs.msg import Key
from actuator_msgs.msg import Actuators

INITIAL_X_POSITION = 0
INITIAL_Y_POSITION = 0
INITIAL_Z_POSITION = 0
INITIAL_ROLL_POSITION = 0
INITIAL_PITCH_POSITION = 0
INITIAL_YAW_POSITION = 0

FRONT_TO_REAR_FORCE_RATIO = 1.06847 
FRONT_TO_REAR_VEL_RATIO = FRONT_TO_REAR_FORCE_RATIO ** 0.5

KEY_W = 119
KEY_S = 115
KEY_U = 117
KEY_J = 106

class FlightController(Node):
    def __init__(self):
        super().__init__("flight_controller")
        self.target = {'x'      : INITIAL_X_POSITION,
                       'y'      : INITIAL_Y_POSITION,
                       'z'      : INITIAL_Z_POSITION,
                       'roll'   : INITIAL_ROLL_POSITION,
                       'pitch'  : INITIAL_PITCH_POSITION,
                       'yaw'    : INITIAL_YAW_POSITION,
                       }

        self.key_down = 0

        self.pub_rr = self.create_publisher(
            Actuators,
            "/bebop/command/motor_speed/prop_rr",
            10
        )
        self.pub_rl = self.create_publisher(
            Actuators,
            "/bebop/command/motor_speed/prop_rl",
            10
        )
        self.pub_fr = self.create_publisher(
            Actuators,
            "/bebop/command/motor_speed/prop_fr",
            10
        )
        self.pub_fl = self.create_publisher(
            Actuators,
            "/bebop/command/motor_speed/prop_fl",
            10
        )


        self._sub = self.create_subscription(
            Key,
            "/keydown",
            self.update_controller,
            10
        )

        self.timer_period = 0.02        # 50Hz
        self.timer = self.create_timer(
            self.timer_period,
            self.control_loop
        )
    
    def control_loop(self):
        target = self.target
        # prop_rl, prop_rr, prop_fl, prop_fr = self.get_motors_speed(target)
        rear_motors = Actuators() 
        front_motors = Actuators()
        rear_motors.velocity = [self.target["z"]]
        front_motors.velocity = [FRONT_TO_REAR_VEL_RATIO * self.target["z"]]
        self.pub_rl.publish(rear_motors)
        self.pub_rr.publish(rear_motors)
        self.pub_fl.publish(front_motors)
        self.pub_fr.publish(front_motors)
    
    def publish_speed(self, publisher, value):
        publisher.publish(value)
    
    def get_motors_speed(target: dict):
        pass
        # return prop_rl, prop_rr, prop_fl, prop_fr

    
    def update_controller(self, msg: Key):
        self.key_down = msg.code
        if self.key_down == KEY_W:
            self.target["z"] += 100
        elif self.key_down == KEY_S:
            self.target["z"] -= 100
        elif self.key_down == KEY_U:
            self.target["z"] += 10
        elif self.key_down == KEY_J:
            self.target["z"] -= 10


def main(args = None):
    rclpy.init(args=args)

    flight_controller = FlightController()

    try:
        rclpy.spin(flight_controller)
    except KeyboardInterrupt:
        pass
    finally:
        flight_controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == "__main__":
    main()
