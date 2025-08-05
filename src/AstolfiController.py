import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, TwistStamped
from tf2_msgs.msg import TFMessage
import numpy as np
import math


class AstolfiController(Node):
    def __init__(self):
        super().__init__('astolfi')

        self.cmd_pub = self.create_publisher(TwistStamped, '/mec_dr_cont/reference', 10)
        self.target_sub = self.create_subscription(Vector3, '/motion_target_pose', self.cb_target, 10)
        self.subodom_ = self.create_subscription(TFMessage, '/mec_dr_cont/tf_odometry', self.cb_tf_odometry, 10)
        self.substop_ = self.create_subscription(Bool, '/motion_abort', self.cb_stop, 10)
        self.subexe_  = self.create_subscription(Bool,'/motion_execute',self.cb_exe,10)
        self.submaxvel_  = self.create_subscription(Float32,'/motion_maxvel',self.cb_maxvel,10)

        self.get_logger().info("Astolfi Node Started")

        # State
        self.currX = 0.0
        self.currY = 0.0
        self.currT = 0.0
        self.enabled = False
        self.target_x = 0.0
        self.target_y = 0.0
        self.first = False
        self.oncestop = False
        self.target_theta = 0.0
        self.max_velocity = 0.1
        self.hard_max_velocity = 0.1


    def cb_exe(self,msg):
        self.enabled = msg.data

    def cb_maxvel(self,msg):
        if msg.data>0 and msg.data < self.hard_max_velocity:
            self.max_velocity = msg.data
        else:
            self.get_logger().error("Max Velocity value not allowed (<0 or to high (0.1))")


    def cb_stop(self, msg):
        if msg.data:
            self.enabled = not msg.data
  
   

    def cb_target(self, msg):

        self.first = True
        

        self.target_x = msg.x #[m]
        self.target_y = msg.y #[m]
        self.target_theta = msg.z *3.14159/180.0 # z used for theta

        self.get_logger().info(
            f"Target received -> x: {self.target_x:.2f}, y: {self.target_y:.2f}, theta: {self.target_theta:.2f} rad"
        )
        if not self.enabled:
            return
        self.astolfi()



    def cb_tf_odometry(self, msg):

        if self.first:

            try:
                transform = msg.transforms[0]

                self.currX = transform.transform.translation.x #[m]
                self.currY = transform.transform.translation.y #[m]
                q = transform.transform.rotation
                self.currT = self.quaternion_to_yaw(q) #[rad]

                if self.enabled:
                    self.astolfi()
                    self.oncestop = False
                else:
                    if not self.oncestop:
                        self.pub_command(0.0,0.0,0.0)
                        self.oncestop = True
                

            except Exception as e:
                self.get_logger().error(f"TF Odometry callback error: {e}")

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def astolfi(self):
        # Gains
        kp = 5.0
        ka = 5.0
        kb = 5.0

        try:
            pos = np.array([self.currX, self.currY]) #[m]
            goal = np.array([self.target_x, self.target_y]) #[m]
            theta = self.currT
            goal_theta = self.target_theta

            delta = goal - pos #[m]
            pho = np.linalg.norm(delta)

            if pho < 0.03: #[3 cm]
                delta = np.zeros(2)
                pho = 0.0

            R = np.array([
                [math.cos(theta), math.sin(theta)],
                [-math.sin(theta), math.cos(theta)]
            ])
            delta_local = R @ delta

            vx = kp * delta_local[0]
            vy = kp * delta_local[1]

            if pho>0.1: #10 cm
                alpha = self.normalize_angle(math.atan2(delta[1], delta[0]) - theta)
            else:
                alpha = 0
            beta = self.normalize_angle(goal_theta - theta - alpha)

            omega = ka * alpha + kb * beta
            # omega = 0


            # Saturate velocity
            vx = np.clip(vx, -self.max_velocity, self.max_velocity)
            vy = np.clip(vy, -self.max_velocity, self.max_velocity)
            omega = np.clip(omega, -self.max_velocity, self.max_velocity)

            total = math.sqrt(vx**2 + vy**2 + omega**2)
            if total > self.max_velocity:
                scale = self.max_velocity / total
                vx *= scale
                vy *= scale
                omega *= scale

            if abs(vx) < 1e-3: vx = 0.0
            if abs(vy) < 1e-3: vy = 0.0
            if abs(omega) < 1e-3: omega = 0.0
            
            self.pub_command(vx,vy,omega)
            self.get_logger().info(
            f"[Astolfi] vx: {vx:.2f}, vy: {vy:.2f}, omega: {omega:.2f} delta_local: {delta_local}, currTheta: {self.currT:.2f}, alpha: {alpha:.2f}, beta: {beta:.2f}"
            )
            
        except Exception as e:
            self.get_logger().error(f"Astolfi failed: {e}")

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def pub_command(self,vx,vy,omega):
    
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'
        twist_msg.twist.linear.x = vx
        twist_msg.twist.linear.y = vy
        twist_msg.twist.angular.z = omega

        self.cmd_pub.publish(twist_msg)

        


def main(args=None):
    rclpy.init(args=args)
    node = AstolfiController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("👋 Shutting down Astolfi node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
