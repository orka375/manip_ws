import rclpy
from rclpy.node import Node
import smach.log
from std_msgs.msg import Bool, String
import smach
import threading
import time
import random
import logging
from std_srvs.srv import SetBool

def NO(s):
    pass

def YES(s):
    if "State machine transitioning" not in s:
        print(s)

smach.log.set_loggers(YES,YES,YES,YES)



# -------------------------
# State: SystemOff
# -------------------------
class SystemOff(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, 
                             outcomes=['to_SystemStartingUp', 'keep'],
                             input_keys=['last_state','css'],
                             output_keys=['last_state'])
        self.node = node
        self.FirstEntry=False
        self.logger = node.get_logger(); 




    def execute(self, userdata):
        
        #--- Reentry
        if userdata.last_state=='SystemOff':
            pass


        #--- from other State
        if userdata.last_state!='SystemOff':
            userdata.last_state='SystemOff'
            self.logger.info("State: SystemOff")

        #--- 1. entry
        if not self.FirstEntry:
            self.FirstEntry = True
            userdata.last_state='SystemOff'
            self.logger.info("State: SystemOff")
        
        if userdata.css:
            return 'to_SystemStartingUp'
        else:
            return 'keep'

# -------------------------
# State: SystemStartingUp
# -------------------------
class SystemStartingUp(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, 
                             outcomes=['to_SystemOn', 'keep'],
                             input_keys=['last_state'],
                             output_keys=['last_state'])
        self.node = node
        self.FirstEntry=False
        self.logger = node.get_logger(); self.logger.info("State: SystemStartingUp")
       

    def execute(self, userdata):
        
        
        #--- Reentry
        if userdata.last_state=='SystemStartingUp':
            pass


        #--- from other State
        if userdata.last_state!='SystemStartingUp':
            userdata.last_state='SystemStartingUp'
            self.logger.info("State: SystemStartingUp")


        #--- 1. entry
        if not self.FirstEntry:
            self.FirstEntry = True
            userdata.last_state='SystemStartingUp'
            self.logger.info("State: SystemStartingUp")

        time.sleep(5)
        return 'to_SystemOn'

# -------------------------
# State: SystemOn
# -------------------------
class SystemOn(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, 
                             outcomes=['to_SystemOff', 'to_SystemMotorPowerOn', 'keep'],
                             input_keys=['enablebase','last_state','css'],
                             output_keys=['last_state'])
        self.node = node
        self.FirstEntry=False
        self.logger = node.get_logger(); 

        self.client = node.create_client(SetBool, '/set_PowerBase')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.warn("Waiting for /enable_base service...")


            
    def execute(self, userdata):
        

        
        #--- Reentry
        if userdata.last_state=='SystemOn':
            pass


        #--- from State SystemMotorPowerOn
        if userdata.last_state=='SystemMotorPowerOn':
            userdata.last_state='SystemOn'
            self.logger.info("State: SystemOn")

            request = SetBool.Request()
            request.data = False

            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)

            if future.result() and future.result().success:
                self.logger.info("Service call succeeded")
                return 'keep'
            else:
                self.logger.error("Service call failed")
                return 'to_SystemOn'

        #--- 1. entry
        if not self.FirstEntry:
            self.FirstEntry = True
            userdata.last_state='SystemOn'
            self.logger.info("State: SystemOn")


        
        if userdata.enablebase:
            return 'to_SystemMotorPowerOn'
        elif not userdata.css:
            return 'to_SystemOff'
        else:
            return 'keep'

# -------------------------
# State: SystemMotorPowerOn
# -------------------------
class SystemMotorPowerOn(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, 
                             outcomes=['to_SystemOn', 'to_SystemMoving', 'keep'],
                             input_keys=['moving', 'enablebase','last_state'],
                             output_keys=['last_state'])
        self.node = node
        self.FirstEntry=False
        self.logger = node.get_logger(); 


        self.client = node.create_client(SetBool, '/set_PowerBase')

        # Wait until the service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.warn("Waiting for /enable_base service...")

    def execute(self, userdata):
        
 
        #--- Reentry
        if userdata.last_state=='SystemMotorPowerOn':
            pass


        #--- from SystemOn
        if userdata.last_state=='SystemOn':
            self.logger.info("State: SystemMotorPowerOn")
            userdata.last_state='SystemMotorPowerOn'

            request = SetBool.Request()
            request.data = True

            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)

            if future.result() and future.result().success:
                self.logger.info("Service call succeeded")
                return 'keep'
            else:
                self.logger.error("Service call failed")
                return 'to_SystemOn'

        #--- from other State
        if userdata.last_state=='SystemMoving':
            self.logger.info("State: SystemMotorPowerOn")
            userdata.last_state='SystemMotorPowerOn'

            


        #--- 1. entry
        if not self.FirstEntry:
            self.FirstEntry = True
            userdata.last_state='SystemMotorPowerOn'
            self.logger.info("State: SystemMotorPowerOn")



        if not userdata.enablebase:
            return 'to_SystemOn'
        


        if userdata.moving:
            return 'to_SystemMoving'
        return 'keep'

# -------------------------
# State: SystemMoving
# -------------------------
class SystemMoving(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, 
                             outcomes=['to_SystemMotorPowerOn', 'keep'],
                             input_keys=['moving','last_state','enablebase'],
                             output_keys=['last_state'])
        self.node = node
        self.FirstEntry=False
        self.logger = node.get_logger(); 

    def execute(self, userdata):
        
        #--- Reentry
        if userdata.last_state=='SystemMoving':
            pass


        #--- from other State
        if userdata.last_state!='SystemMoving':
            userdata.last_state='SystemMoving'
            self.logger.info("State: SystemMoving")

        #--- 1. entry
        if not self.FirstEntry:
            self.FirstEntry = True
            userdata.last_state='SystemMoving'
            self.logger.info("State: SystemMoving")
        


        if not userdata.moving:
            return 'to_SystemMotorPowerOn'

        elif not userdata.enablebase:
            return 'to_SystemMotorPowerOn'
            #TODO Call moving topic to set it to false
        else:
            return 'keep'



#TODO USAGE OF EMERGERNY STATE
#BUG


# -------------------------
# Emergency State
# -------------------------
class Emergency(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, 
                             outcomes=['to_SystemOff', 'keep'],
                             input_keys=['e_reset','last_state'],
                             output_keys=['last_state'])
        self.node = node
        self.FirstEntry=False
        self.logger = node.get_logger(); 
    



    def execute(self, userdata):

        #--- Reentry
        if userdata.last_state=='Emergency':
            pass


        #--- from other State
        if userdata.last_state!='Emergency':
            userdata.last_state='Emergency'
            self.logger.warn("🚨 Emergency triggered!")

        if userdata.e_reset:
            return 'to_SystemOff'
        else:
            return 'keep'

# -------------------------
# Shared Node
# -------------------------
class SharedNode(Node):
    def __init__(self):
        super().__init__('smach_topic_listener')
        self.base_enabled = False
        self.abort_motion = False
        self.moving = False
        self.e_reset = False
        self.css = False
        self.current_state = 'Unknown'

        self.state_pub_timer = self.create_timer(0.5, self.publish_current_state)

        self.state_pub = self.create_publisher(String, 'sm_state', 10)


        self.create_subscription(Bool, '/sm_enable_base', self.base_callback, 10)
        self.create_subscription(Bool, '/sm_abort_motion', self.abort_callback, 10)
        self.create_subscription(Bool, '/sm_robot_is_moving', self.moving_callback, 10)
        self.create_subscription(Bool, '/sm_error_reset', self.errorreset_callback, 10)
                
        self.create_subscription(Bool,'/current_system_state',self.cb_css,10)

    def cb_css(self,msg):
        self.css = msg.data

    def publish_current_state(self):
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)
        # self.get_logger().info(f"📢 Published current state: {self.current_state}")


    def base_callback(self, msg):
        self.base_enabled = msg.data

    def abort_callback(self, msg):
        self.abort_motion = msg.data

    def moving_callback(self, msg):
        self.moving = msg.data

    def errorreset_callback(self, msg):
        self.e_reset = msg.data

# -------------------------
# Main Function
# -------------------------
def main():
    rclpy.init()
    node = SharedNode()

    sm = smach.StateMachine(outcomes=['done'])
    sm.userdata.enablebase = False
    sm.userdata.moving = False
    sm.userdata.abort = False
    sm.userdata.e_reset = False
    sm.userdata.last_state = 'Unkown'
    sm.userdata.css = False

    with sm:
        smach.StateMachine.add('SystemOff', SystemOff(node),
            transitions={'to_SystemStartingUp': 'SystemStartingUp', 
                         'keep': 'SystemOff'}
                         )

        smach.StateMachine.add('SystemStartingUp', SystemStartingUp(node),
            transitions={'to_SystemOn': 'SystemOn',
                          'keep': 'SystemStartingUp'}
                          )

        smach.StateMachine.add('SystemOn', SystemOn(node),
            transitions={'to_SystemOff': 'SystemOff',
                         'to_SystemMotorPowerOn': 'SystemMotorPowerOn',
                         'keep': 'SystemOn'}
                         )

        smach.StateMachine.add('SystemMotorPowerOn', SystemMotorPowerOn(node),
            transitions={'to_SystemOn': 'SystemOn',
                         'to_SystemMoving': 'SystemMoving',
                         'keep': 'SystemMotorPowerOn'},
                         )

        smach.StateMachine.add('SystemMoving', SystemMoving(node),
            transitions={'to_SystemMotorPowerOn': 'SystemMotorPowerOn',
                          'keep': 'SystemMoving'}
                         )

        smach.StateMachine.add('Emergency', Emergency(node),
            transitions={'to_SystemOff': 'SystemOff',
                          'keep': 'Emergency'}
                          )

    # Background thread for ROS spinning + updating userdata
    def ros_spin_thread():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            sm.userdata.enablebase = node.base_enabled
            sm.userdata.moving = node.moving
            sm.userdata.abort = node.abort_motion
            sm.userdata.e_reset = node.e_reset
            sm.userdata.css = node.css

            node.current_state = sm.userdata.last_state  # Take from userdata
            

    threading.Thread(target=ros_spin_thread, daemon=True).start()

    # Start SMACH
    sm.execute()

    node.destroy_node()
    rclpy.shutdown()
    print("\n✅ State machine finished.")

if __name__ == '__main__':
    main()
