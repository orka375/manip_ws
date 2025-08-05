import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import subprocess
from std_msgs.msg import Bool, String
import signal
import os


class LaunchSelector(Node):
    def __init__(self):
        super().__init__('simreal_selector')

        self.get_logger().info('✅ LaunchSelector node initialized')


        self.current_process = None
        self.current_system_state = False
        self.current_system = "None"

        self.current_command = None  # ← ADD THIS
        self.current_process_type = None  # ← ADD THIS
        self.curr_sm_state = "None"  # ← ADD THIS


        self.launch_map = {
            'ActivateRealSystem': 'launch_robot.launch.py',  # REAL
            'ActivateSimSystem': 'launch_sim.launch.py'    # SIM
        }

        # Create both services
        self.create_service(SetBool, 'ActivateRealSystem', self.activate_real_cb)
        self.create_service(SetBool, 'ActivateSimSystem', self.activate_sim_cb)

        self.pub_curr_sys_state = self.create_publisher(Bool, '/current_system_state', 10)
        self.pub_curr_sys= self.create_publisher(String, '/current_system', 10)
        
        self.sub_curr_sm_state = self.create_subscription(String,'/sm_state',self.cb_sm_state,10)

        self.get_logger().info('✅ Services ready: /ActivateRealSystem and /ActivateSimSystem')

    def cb_sm_state(self, msg):
        self.curr_sm_state  = msg.data


    def activate_real_cb(self, request, response):
        return self._handle_launch_request(request.data, response, label='ActivateRealSystem')

    def activate_sim_cb(self, request, response):
        return self._handle_launch_request(request.data, response, label='ActivateSimSystem')


    def _handle_launch_request(self, command: bool, response: SetBool.Response, label: str):

        response = SetBool.Response()
        launch_file = self.launch_map.get(label)
        # if label=='ActivateSimSystem':
        state = True if command else False
        

        self.get_logger().info(f"[{label}] Requested: System: {state}")
       
        try: 
            if 'SystemMoving' in self.curr_sm_state or 'SystemMotorPowerOn' in self.curr_sm_state or 'SystemStartingUp' in self.curr_sm_state :
                self.get_logger().warning(f"System change currently not allowed, StateMachine in not-allowed state")
            else:
                if self.current_process is not None:
                    msg = Bool()
                    msg.data = False
                    self.pub_curr_sys_state.publish(msg)

                    msg = String()
                    msg.data = "NONE"
                    self.pub_curr_sys.publish(msg)


                    self.get_logger().info(f"🛑 Terminating previous launch: {self.current_process}")
                    os.killpg(os.getpgid(self.current_process.pid), signal.SIGINT)
                    self.current_process.wait()

                    self.get_logger().info(f"✅ Terminated: {self.current_process}")
                    self.current_process = None
                    self.current_process_type = None

                try:
                    if state:
                        msg = Bool()
                        msg.data = True
                        self.pub_curr_sys_state.publish(msg)

                        msg = String()
                        if 'Real' in label:
                            msg.data = "REAL"
                        else:
                            msg.data = "SIM"
                        self.pub_curr_sys.publish(msg)

                        self.get_logger().info(f"🚀 Launching: {launch_file}")
                        self.current_process = subprocess.Popen(
                            ['ros2', 'launch', 'youbot', launch_file],
                            preexec_fn=os.setsid
)

                        self.current_command = state
                        response.success = True
                        response.message = f"Launched: {state} via {label}"
                except Exception as e:
                    msg = f"🔥 Failed to launch: {e}"
                    self.get_logger().error(msg)
                    response.success = False
                    response.message = msg

                return response
        except Exception as e:
            msg = f"🔥 Failed to change System: {e}"
            self.get_logger().error(msg)
            response.success = False
            response.message = msg


def main(args=None):
    rclpy.init(args=args)
    node = LaunchSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.current_process:
            node.get_logger().info("🛑 Shutting down current process before exit...")
            node.current_process.send_signal(signal.SIGINT)
            node.current_process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
