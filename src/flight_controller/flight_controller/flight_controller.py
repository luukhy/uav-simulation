import rclpy 
import math
from rclpy.node import Node 
from geometry_msgs.msg import Twist
from actuator_msgs.msg import Actuators
from sensor_msgs.msg import Imu
from flight_controller import pid

INITIAL_X_VEL = 0
INITIAL_Y_VEL = 0
INITIAL_Z_VEL = 0
INITIAL_THROTTLE = 0
INITIAL_ROLL_POSITION = 0
INITIAL_PITCH_POSITION = 0
INITIAL_YAW_POSITION = 0

FRONT_TO_REAR_FORCE_RATIO = 1.06847 
# FRONT_TO_REAR_VEL_RATIO = FRONT_TO_REAR_FORCE_RATIO ** 0.5
FRONT_TO_REAR_VEL_RATIO = 1

class FlightController(Node):
    def __init__(self):
        super().__init__("flight_controller")
        self.max_tilt = 0.4
        self.target_state = {   'x_vel' : INITIAL_X_VEL,
                                'y_vel' : INITIAL_Y_VEL,
                                'z_vel' : INITIAL_Z_VEL,
                            }
        
        self.current_state= {   'x_vel' : INITIAL_X_VEL,
                                'y_vel' : INITIAL_Y_VEL,
                                'z_vel' : INITIAL_Z_VEL,
                            }
        hover_speed = 387.2
        self.hover_speed_front = FRONT_TO_REAR_VEL_RATIO * hover_speed
        self.hover_speed_rear = hover_speed
                       
        kp_pitch_roll = 100.0
        ki_pitch_roll = 5.0  
        kd_pitch_roll = 30.0 

        kp_yaw = 40.0
        ki_yaw = 0.0
        kd_yaw = 0.0

        self.pid_pitch  = pid.PID(kp_pitch_roll, ki_pitch_roll, kd_pitch_roll, i_max=0.1 * hover_speed)
        self.pid_roll   = pid.PID(kp_pitch_roll * 0.8, ki_pitch_roll, kd_pitch_roll, i_max=0.1 * hover_speed)
        self.pid_yaw    = pid.PID(kp_yaw, ki_yaw, kd_yaw)

        self.key_down = 0

        # subscribers 
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
        )
        self.sub_imu = self.create_subscription(
            Imu,
            "/imu",
            self.imu_callback,
            10
        )
        self.dt = 0.02        # 50Hz
        self.timer = self.create_timer(
            self.dt,
            self.control_loop
        )

        # publisers
        self.pub_rr = self.create_publisher(Actuators, "/bebop/command/motor_speed/prop_rr", 10)
        self.pub_rl = self.create_publisher( Actuators, "/bebop/command/motor_speed/prop_rl", 10)
        self.pub_fr = self.create_publisher( Actuators, "/bebop/command/motor_speed/prop_fr", 10)
        self.pub_fl = self.create_publisher( Actuators, "/bebop/command/motor_speed/prop_fl", 10)
    
    def control_loop(self):
        pitch_output = self.pid_pitch.calculate(self.target_state['pitch'], 
                                                self.current_state['pitch'], 
                                                self.dt)
        roll_output  = self.pid_roll.calculate(self.target_state['roll'], 
                                               self.current_state['roll'], 
                                               self.dt)
        yaw_output   = self.pid_yaw.calculate(self.target_state['yaw'], 
                                              self.current_state['yaw'],
                                              self.dt)

        # print(f"throttle: {self.target_state['throttle']}, pitch_output: {pitch_output}, roll_output: {roll_output}, yaw_output: {yaw_output}")

        
        # limit = 400.0
        motor_rr = (self.hover_speed_rear + self.target_state['throttle']
                + pitch_output - roll_output - yaw_output)
 
        
        motor_rl = (self.hover_speed_rear + self.target_state['throttle'] 
                + pitch_output + roll_output + yaw_output)
        
        motor_fr = (self.hover_speed_front + self.target_state['throttle'] 
                - pitch_output - roll_output + yaw_output)
        
        motor_fl = (self.hover_speed_front + self.target_state['throttle'] 
                - pitch_output + roll_output - yaw_output)

        self.publish_motor_speed(self.pub_rr, motor_rr)
        self.publish_motor_speed(self.pub_rl, motor_rl)
        self.publish_motor_speed(self.pub_fr, motor_fr)
        self.publish_motor_speed(self.pub_fl, motor_fl)
    
    
    def cmd_vel_callback(self, msg: Twist):
        self.target_state['pitch'] = -1.0 * self.max_tilt * msg.linear.x
        self.target_state['roll']  = 1.0 * self.max_tilt * msg.linear.y
        self.target_state['yaw']  = msg.angular.z
        self.target_state['throttle'] = msg.linear.z * 40.0
    
    def imu_callback(self, msg: Imu):
        quat = msg.orientation
        sinr_cosp = 2 * (quat.w * quat.x + quat.y * quat.z)
        cosr_cosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y)
        self.current_state['roll'] = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (quat.w * quat.y - quat.z * quat.x)
        if abs(sinp) >= 1:
            self.current_state['pitch'] = math.copysign(math.pi / 2, sinp)
        else:
            self.current_state['pitch'] = math.asin(sinp)

        self.current_state['yaw'] = msg.angular_velocity.z
    
    def publish_motor_speed(self, publisher, value: float):
        msg = Actuators()
        msg.velocity = [float(max(0.0, value))] 
        publisher.publish(msg)
    

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
