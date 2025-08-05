import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32, Int32
from opcua import ua
from src.OpcUaClient import OpcuaClientNode
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String, Bool, Int32, Int64, Float32, Float64
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool  # Import SetBool service
import math
import transformations
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState
import datetime
from geometry_msgs.msg import Vector3, TwistStamped



import threading
import time
import os
import re
import glob

# Mapping from TwinCAT types to ROS messages and OPC UA types
type_mapping = {
    'BOOL': (Bool, ua.VariantType.Boolean),
    'STRING': (String, ua.VariantType.String),
    'REAL': (Float32, ua.VariantType.Float),
    'LREAL': (Float32, ua.VariantType.Float),
    'INT': (Int32, ua.VariantType.Int16),
    'DINT': (Int32, ua.VariantType.Int32),
    'BYTE': (Int32, ua.VariantType.Byte),
    'WORD': (Int32, ua.VariantType.UInt16),
    'DWORD': (Int32, ua.VariantType.UInt32)
}



class ROBridge(Node):
    def __init__(self):
        super().__init__('ro_bridge')

        self.lock = threading.Lock()
        self.client = OpcuaClientNode()
        self.lastupload = time.time()


        #==============OPC TO ROS ========================================================

        #SUBSCRIBE TO COMMANDS
        self.OPC_to_ROS = [
            'OPCUA.MOTIONCMD',
            'OPCUA.SYSTEMCMD',
        ]

        #MAPPING OF TOPICS TO PUBLISH
        self.ROS_to_OPC = {
            "OPCUA.SYSTEMSTATE.HeartBeat":"/my_heart_beat",
            "OPCUA.SYSTEMSTATE.SystemSafetyLevel":"/sm_state",
            "OPCUA.SYSTEMSTATE.ActivSystem":"/current_system",
            "OPCUA.SYSTEMSTATE.BatteryVoltage":"/vmeter_voltage"
    
        }

        #PUBLISHERS TO ROS
        self.motion_unit_pub = self.create_publisher(String, '/motion_unit', 10)
        self.motion_maxvel_pub = self.create_publisher(Float32, '/motion_maxvel', 10)
        self.motion_execute_pub = self.create_publisher(Bool, '/motion_execute', 10)
        self.motion_abort_pub = self.create_publisher(Bool, '/motion_abort', 10)
        self.motion_target_pub = self.create_publisher(Vector3, '/motion_target_pose', 10)



        # Internal storage for values
        self.target_x = None
        self.target_y = None
        self.target_yaw = None

        self.last_target_x = None
        self.last_target_y = None
        self.last_target_yaw = None


        #==============ROS TO OPC ========================================================

        #Actual Subscriptions
        self.subs_OR = []
        self.subs_RO={}

        



        #Advanced ROS2 Subscriptions
        self.joint_name_map = {
            'wheel_joint_fl': 'FL',
            'wheel_joint_fr': 'FR',
            'wheel_joint_bl': 'BL',
            'wheel_joint_br': 'BR'
        }

        
        self.subJointStateSucceeded  = False
        try:   
            self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
            self.subJointStateSucceeded = True
        except:
            pass


        self.subOdomSucceeded  = False
        try:   
            self.create_subscription(Odometry, '/mec_dr_cont/odometry', self.odom_callback, 10)
            self.subOdomSucceeded = True
        except:
            pass
        


        #--------------Execute--------------
        self.Sub_OPC()
        self.Scan_Sub_Topics()

        # Every 10 seconds, rescan for new topics
        self.topic_scan_timer = self.create_timer(3.0, self.Scan_Sub_Topics)




#region OPCUA TO ROS       
    
    def Sub_OPC(self):
        time.sleep(4) # Wait for Client is setup
        for opcnode in self.OPC_to_ROS:
            self.client.subscribe_to_object_recursive(opcnode,self.callback_changed_opc_var)


    def callback_changed_opc_var(self, node, value):
        # self.get_logger().info(f"📥 [{value}] -> {node}")

        node_id_str = node.nodeid.to_string()
   

        if "SYSTEMCMD" in node_id_str:
            self.callback_systemcmd(node_id_str,value)

        if "MOTIONCMD" in node_id_str:
            self.callback_motioncmd(node_id_str,value)



    def callback_systemcmd(self, nodeid, value):
        if 'EnableBase' in nodeid:
            publisher_ = self.create_publisher(Bool, '/sm_enable_base', 10)
            msg = Bool()
            msg.data = value
            publisher_.publish(msg)

        elif 'EnableArm' in nodeid:
            self.get_logger().warning('EnableArm Callback from OPC: Waiting to be implemented')
            # TODO: Implement logic

        elif 'ActivateRealSystem' in nodeid:
            try:
                pass
                # self.call_setbool_service('ActivateRealSystem', value)
            except: pass

        elif 'ActivateSimSystem' in nodeid:
            try:
                pass
                # self.call_setbool_service('ActivateSimSystem', value)
            except: pass

        elif 'JoyStickMode' in nodeid:
            import subprocess
            if value:
                if getattr(self, 'joyprocess', None) is None:
                    self.joyprocess = subprocess.Popen(['ros2', 'launch', 'youbot', 'joystick.launch.py'])
                    self.get_logger().info('🎮 Joystick launched.')
            else:
                if getattr(self, 'joyprocess', None) is not None:
                    self.joyprocess.terminate()
                    self.joyprocess.wait()
                    self.joyprocess = None
                    self.get_logger().info('🛑 Joystick process terminated.')



    def call_setbool_service(self, service_name, value):
        cli = self.create_client(SetBool, service_name)

        # while not cli.wait_for_service(timeout_sec=2.0):
        #     self.get_logger().warn(f'Waiting for service {service_name}...')

        req = SetBool.Request()
        req.data = value

        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result.success:
            self.get_logger().info(f"✅ {service_name} called successfully: {result.message}")
        else:
            self.get_logger().error(f"❌ {service_name} failed: {result.message}")




    def callback_motioncmd(self, nodeid, value):
        if 'TargetUnit' in nodeid:
            msg = String()
            msg.data = value
            self.motion_unit_pub.publish(msg)

        elif 'setVelocity' in nodeid:
            msg = Float32()
            msg.data = value
            self.motion_maxvel_pub.publish(msg)
        
        elif 'Execute' in nodeid:
            msg = Bool()
            msg.data = value
            self.motion_execute_pub.publish(msg)
            self.get_logger().info(f'{value}')
        
        elif 'Abort' in nodeid:
            msg = Bool()
            msg.data = value
            self.motion_abort_pub.publish(msg)

        elif 'Target.X' in nodeid:
            self.target_x = value
            self.check()
        elif 'Target.YAW' in nodeid:
            self.target_yaw = value
            self.check()
        elif 'Target.Y' in nodeid:
            self.target_y = value
            self.check()
        
        elif 'TargetJoints' in nodeid:
            self.get_logger().warning('TargetJoints Callback from OPC: Waiting to be implemented')
            


    def check(self):
        self.get_logger().info(f"Target: {self.target_x} {self.target_y} {self.target_yaw}")

        if (self.target_x is not None) and (self.target_y is not None) and (self.target_yaw is not None):
            # Check if any value has changed
            if (self.target_x != self.last_target_x or 
                self.target_y != self.last_target_y or 
                self.target_yaw != self.last_target_yaw):
    
                # Prepare and publish message
                target_msg = Vector3()
                target_msg.x = float(self.target_x)
                target_msg.y = float(self.target_y)
                target_msg.z = float(self.target_yaw)  # YAW mapped to z

                self.motion_target_pub.publish(target_msg)
                self.get_logger().info(f"Published Vector3: {target_msg}")

                # Update last published values
                self.last_target_x = self.target_x
                self.last_target_y = self.target_y
                self.last_target_yaw = self.target_yaw
            else:
                self.get_logger().info("No change in target values; not publishing.")


#endregion

#region ROS TO OPCUA


    def callback_changed_ros_topic(self, msg, topic_name):
        if topic_name not in self.subs_RO:
            self.get_logger().warn(f"No mapping found for topic '{topic_name}'")
            return

        try:
            with self.lock:
                config = self.subs_RO[topic_name]
                node = self.client.client.get_node(config['node_id'])  # Access underlying OPC UA client
                value = msg.data
                variant = ua.Variant(value, config['type'])
                data_value = ua.DataValue()
                data_value.Value = variant
                node.set_value(data_value)

                # self.get_logger().info(f"✅ [{topic_name}] → OPC UA [{config['node_id']}] = {value}")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed to write from topic '{topic_name}' to OPC UA: {e}")


    def subscribe_to_ros_topic(self, topic_name, msg_type, callback):
        def wrapped_callback(msg):
            callback(msg, topic_name)

        self.create_subscription(
            msg_type,
            topic_name,
            wrapped_callback,
            10
        )

        self.get_logger().info(f"📡 Subscribed to ROS topic '{topic_name}'")
    


    def Scan_Sub_Topics(self):

        #Advanced
        if not self.subOdomSucceeded:
            try:   
                self.create_subscription(Odometry, '/mec_dr_cont/tf_odometry', self.odom_callback, 10)
                self.subOdomSucceeded = True
            except:
                pass
        
        if not self.subJointStateSucceeded:
            try:   
                self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
                self.subJointStateSucceeded = True
            except:
                pass

        #Normal

        topic_list = self.get_topic_names_and_types()


        for topic_name, types in topic_list:
            if topic_name in self.subs_RO:
                continue  # already registered

            if topic_name not in self.ROS_to_OPC.values():
                continue  # not in lookup table
            
   

            msg_type_str = types[0]

            if msg_type_str == 'std_msgs/msg/String':
                msg_type = String
                variant_type = ua.VariantType.String
            elif msg_type_str == 'std_msgs/msg/Bool':
                msg_type = Bool
                variant_type = ua.VariantType.Boolean
            elif msg_type_str == 'std_msgs/msg/Int32':
                msg_type = Int32
                variant_type = ua.VariantType.Int32
            elif msg_type_str == 'std_msgs/msg/Int64':
                msg_type = Int64
                variant_type = ua.VariantType.Int64
            elif msg_type_str == 'std_msgs/msg/Float32':
                msg_type = Float32
                variant_type = ua.VariantType.Float
            elif msg_type_str == 'std_msgs/msg/Float64':
                msg_type = Float64
                variant_type = ua.VariantType.Double
            else:
                self.get_logger().warn(f"❌ Topic '{topic_name}' has unsupported type: {msg_type_str}")
                continue

            # Find the corresponding OPC UA variable name (key in lookup table)
            opcua_var_name = next((k for k, v in self.ROS_to_OPC.items() if v == topic_name), None)
            if opcua_var_name is None:
                self.get_logger().warn(f"⚠️ No OPC UA mapping found for topic '{topic_name}'")
                continue

            node_id = f'ns=4;s={opcua_var_name}'

            # Register mapping
            self.subs_RO[topic_name] = {
                'node_id': node_id,
                'type': variant_type,
                'msg_type': msg_type
            }

            # Create subscription
            self.subscribe_to_ros_topic(
                topic_name,
                msg_type,
                self.callback_changed_ros_topic
            )

            self.get_logger().info(f"🔗 Linked topic '{topic_name}' to OPC UA node '{node_id}'")

#endregion



#region Advanced ROS2 Subscriptions


    def joint_state_callback(self, msg):
        try:
            with self.lock:
                for i, name in enumerate(msg.name):
                    if name in self.joint_name_map:
                        suffix = self.joint_name_map[name]
                        pos = msg.position[i] if i < len(msg.position) else 0.0
                        vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
                        self.set_opc_value(f"OPCUA.MOTIONSTATE.currJointPos.{suffix}", pos)
                        self.set_opc_value(f"OPCUA.MOTIONSTATE.currJointVel.{suffix}", vel)
        except Exception as e:
            self.get_logger().warning(f"⚠️ JointState OPC UA write error: {e}")



    def odom_callback(self, msg):
        try:
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            linear = msg.twist.twist.linear
            angular = msg.twist.twist.angular
            yaw = transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]

            with self.lock:
                self.set_opc_value("OPCUA.MOTIONSTATE.currPos.X", pos.x)
                self.set_opc_value("OPCUA.MOTIONSTATE.currPos.Y", pos.y)
                self.set_opc_value("OPCUA.MOTIONSTATE.currPos.YAW", yaw)
                self.set_opc_value("OPCUA.MOTIONSTATE.currVel.X", linear.x)
                self.set_opc_value("OPCUA.MOTIONSTATE.currVel.Y", linear.y)
                self.set_opc_value("OPCUA.MOTIONSTATE.currVel.YAW", angular.z)
        except Exception as e:
            self.get_logger().warning(f"⚠️ Odometry OPC UA write error: {e}")

    def set_opc_value(self, node_path, value):
        node = self.client.client.get_node(f"ns=4;s={node_path}")
        node.set_value(ua.DataValue(ua.Variant(value, ua.VariantType.Float)))

#endregion











def main(args=None):
    rclpy.init(args=args)
    bridge = ROBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
