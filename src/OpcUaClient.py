import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from opcua import Client
from opcua import ua
from std_srvs.srv import Trigger
from std_msgs.msg import String, Bool, Int32, Int64, Float32, Float64

from rosidl_runtime_py.utilities import get_message

from rclpy.topic_endpoint_info import TopicEndpointInfo




import threading
import os   

class OpcuaClientNode(Node):
    def __init__(self):
        super().__init__('opcua_client_node')
        node_name = f'/ns_{self.get_name()}'
        print(node_name)
        self.publisher_ = self.create_publisher(Int32, node_name, 10)

           
        # URI
        self.uri_file_path = "/home/fabian/youbot_ws/opcua_uri.txt" #TODO Change absolut path
        self.server_url = self.load_last_uri(default="opc.tcp://192.168.150.3:4840")
        
        self.client = Client(self.server_url)

        #Security
        self.client.set_security_string("Basic256Sha256,SignAndEncrypt,/home/nbf2/youbot_ws/cert/client_cert.der,/home/nbf2/youbot_ws/cert/client_key.pem") #TODO Change absolut path
        self.client.set_user("Admin")
        self.client.set_password("1")
        
        self.connected = False
        self.lock = threading.Lock()

        # ROS2 
        self.status_pub = self.create_publisher(Bool, 'opcua_connection_status', 10)                            # Publisher for connection status
        self.uri_sub = self.create_service(Trigger, 'set_opcua_set_uri', self.set_uri_service_callback)                 # Subscriber to update the server URI #TODO ACCEPT STRING
        self.connect_service = self.create_service(Trigger, 'try_opcua_connect', self.connect_to_server)        # Service to manually trigger connection

        # Subscriptions to OPCUA
        self.mysubscriptions = {}  # Store active subscriptions to avoid duplicates
        self.opcua_subscription = None  # The main OPC UA subscription object
        self.topic_to_opcua_node={}

        # Timer to periodically attempt (re)connection
        self.reconnect_timer = self.create_timer(8.0, self.connection_check)
    


        self.get_logger().info(f"Initialized with OPC UA URI: {self.server_url}")
        self.connect_to_server()



#region Methods for Subscription to OPCUA Server Objects and Variables

    def subscribe_to_opcua_variable_callback(self, request, response):
        node_id = request.data
        self.get_logger().info(f"🔧 Service called to subscribe to OPC UA node: {node_id}")

        def callback(node, value):
            self.get_logger().info(f"📡 OPC UA update from {node}: {value}")

        self.subscribe_to_variable(node_id, callback)
        response.success = True
        response.message = f"Subscribed to OPC UA node: {node_id}"
        return response



    def subscribe_to_variable(self, node_id_str: str, on_change_callback):
        if not self.connected:
            self.get_logger().warn("Cannot subscribe, not connected to OPC UA server.")
            return

        if not node_id_str.startswith("ns="):
            node_id_str = f"ns=4;s={node_id_str}"  # You might want to make ns configurable later

        if node_id_str in self.mysubscriptions:
            self.get_logger().info(f"Already subscribed to node: {node_id_str}")
            return

        try:
            
            node = self.client.get_node(node_id_str)
            handler = SubHandler(on_change_callback)

            if self.opcua_subscription is None:
                self.opcua_subscription = self.client.create_subscription(1000, handler)

            handle = self.opcua_subscription.subscribe_data_change(node)
            self.mysubscriptions[node_id_str] = handle
            self.get_logger().info(f"✅ Subscribed to data changes for node: {node_id_str}")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed to subscribe to node {node_id_str}: {e}")



    def subscribe_to_object_recursive(self, node_id_str: str, on_change_callback, depth=0):
        if not self.connected:
            self.get_logger().warn("Cannot subscribe, not connected to OPC UA server.")
            return

        if not node_id_str.startswith("ns="):
            node_id_str = f"ns=4;s={node_id_str}"  # You might want to make ns configurable later

        try:
            node = self.client.get_node(node_id_str)
            node_class = node.get_node_class()

            indent = "  " * depth  # for logging nicely

            if node_class == ua.NodeClass.Variable:
                self.get_logger().info(f"{indent}📘 Variable: {node_id_str}")
                self.subscribe_to_variable(node_id_str, on_change_callback)

            elif node_class == ua.NodeClass.Object:
                self.get_logger().info(f"{indent}📂 Object: {node_id_str}")
                children = node.get_children()

                for child in children:
                    try:
                        child_id = child.nodeid.to_string()
                        self.subscribe_to_object_recursive(child_id, on_change_callback, depth=depth+1)
                    except Exception as e:
                        self.get_logger().warn(f"{indent}⚠️ Failed on child: {e}")

            else:
                self.get_logger().info(f"{indent}🔍 Skipped non-object/variable node: {node_id_str} (type: {node_class})")

        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed to recurse node {node_id_str}: {e}")


#endregion





#region Methods for Subscription to ROS2 Topics







    
#endregion



#region Basic Client Functions

    def connection_check(self):
        with self.lock:
            if self.connected:
                try:
                    # self.browse_opcua_nodes()
                    node = self.client.get_node("ns=4;s=OPCUA.ServerHeartBeat")
                    _ = node.get_value()
                except Exception as e:
                    self.get_logger().warn(f"Lost connection to OPC UA server: {e}")
                    self.handle_disconnection()
            else:
                self.connect_to_server()

    def connect_to_server(self):
        if not self.connected:
            try:
                self.get_logger().info(f"Trying to connect to OPC UA server at {self.server_url}")
                self.client.connect()
                
                self.connected = True
                self.publish_status_connection(True)
            except Exception as e:
                self.get_logger().warn(f"Connection attempt failed: {e}, add if needed a new URI via ROS2 topics")
                self.connected = False
                self.publish_status_connection(False)

    def disconnect_from_server(self):
        with self.lock:
            if self.client:
                try:
                    self.client.disconnect()
                    self.get_logger().info("Disconnected from OPC UA server")
                except Exception as e:
                    self.get_logger().warn(f"Error disconnecting: {e}")
                finally:
                    self.handle_disconnection()

    def handle_disconnection(self):
        self.connected = False
        self.publish_status_connection(False)
        # self.client = None
        self.get_logger().info("Handled disconnection event")

    def publish_status_connection(self, status: bool):
        msg = Bool()
        msg.data = status
        self.status_pub.publish(msg)

    def save_uri_to_file(self, uri: str):
        try:
            with open(self.uri_file_path, 'w') as f:
                f.write(uri)
            self.get_logger().info(f"🔒 Saved URI to {self.uri_file_path}")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed to save URI: {e}")

    def set_uri_service_callback(self, request, response):
        self.get_logger().info(f"Service called to set URI: {request.uri}")
        if request.uri != self.server_url:
            self.server_url = request.uri
            self.save_uri_to_file(self.server_url)
            self.disconnect_from_server()
            self.connect_to_server()
        response.success = self.connected
        response.message = "URI updated and connection attempt made."
        return response

    def load_last_uri(self, default: str) -> str:
        if os.path.exists(self.uri_file_path):
            try:
                with open(self.uri_file_path, 'r') as f:
                    uri = f.read().strip()
                    self.get_logger().info(f"🔓 Loaded saved URI: {uri}")
                    return uri
            except Exception as e:
                self.get_logger().warn(f"⚠️ Failed to read saved URI: {e}")
        return default


#endregion






class SubHandler:
    def __init__(self, callback):
        self.callback = callback

    def datachange_notification(self, node, val, data):
        print(f"🔔 Data Change Detected on Node {node}: New Value = {val}")
        self.callback(node, val)






def main(args=None):
    rclpy.init(args=args)
    node = OpcuaClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node")

    finally:
        node.disconnect_from_server()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
