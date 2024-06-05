''' ROS NODE TO CAPTURE STATES AT REAL EXECUTION TIME FOR GRAPH: high level semantic

 TODO: check node output publications with turtlebot3 sim + semantic map. 
 TODO: create another ROS 2 node with igraph to change the state and color depending on the ros topics 

    self.SEMANTIC_MAP_UNKNOWN = False
    self.SEMANTIC_MAP_KNOWN = False
    self.FRONTIER_EXPLORATION = False
'''

import rclpy
from std_msgs.msg import Bool  # Changed from String to Bool
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from rclpy.node import Node
import struct

class States(Node):
    def __init__(self):
        super().__init__('sgraph_state')

        # Initialize class variables
        self.SEMANTIC_MAP_UNKNOWN = True
        self.SEMANTIC_MAP_KNOWN = False
        self.FRONTIER_EXPLORATION = False
        self.FRONTIER_EXPLORATION_ENABLED = False 

        # SEMANTIC_MAP_UNKNOWN - Define the callback function to be executed when a message is received
        self.subscription_semantic_pcl = self.create_subscription(PointCloud2, '/semantic_pcl', self.semantic_map_callback, 10)
        self.subscription_random_exploration = self.create_subscription(Bool, '/map_finished', self.random_exploration_callback, 10)

        # GRAPH STATE PUBLISHERS:
        self.state_publisher_map_unknown = self.create_publisher(Bool, 'SEMANTIC_MAP_UNKNOWN', 10)
        self.state_publisher_map_known = self.create_publisher(Bool, 'SEMANTIC_MAP_KNOWN', 10)
        self.state_publisher_frontier_exploration = self.create_publisher(Bool, 'FRONTIER_EXPLORATION', 10)
        #Create a timer that will publish every timer_period (constant defined bellow) pcl_msg and send tf_broadcaster.sendTransform
        timer_period = 4.0  # seconds
        self.timer = self.create_timer(timer_period, self.semantic_map_check)
    
    def semantic_map_check(self):
        if (self.FRONTIER_EXPLORATION == False) and (self.SEMANTIC_MAP_UNKNOWN == True):
            self.get_logger().info("SEMANTIC_MAP_UNKNOWN1 = True")
            result = Bool()
            result.data = self.SEMANTIC_MAP_UNKNOWN
            self.state_publisher_map_unknown.publish(result)
        if (self.FRONTIER_EXPLORATION_ENABLED == False) and (self.SEMANTIC_MAP_UNKNOWN == False):            
            self.get_logger().info("SEMANTIC_MAP_KNOWN2 = True")          
            self.SEMANTIC_MAP_KNOWN = True
            result = Bool()
            result.data = self.SEMANTIC_MAP_KNOWN
            self.state_publisher_map_known.publish(result)
        if (self.FRONTIER_EXPLORATION_ENABLED == True) and (self.SEMANTIC_MAP_UNKNOWN == True):
            self.get_logger().info("SEMANTIC_MAP_UNKNOWN3 = True")            
            result = Bool()
            result.data = self.SEMANTIC_MAP_UNKNOWN
            self.state_publisher_map_unknown.publish(result)
        if (self.FRONTIER_EXPLORATION_ENABLED == True) and (self.SEMANTIC_MAP_UNKNOWN == False):
            self.get_logger().info("FRONTIER_EXPLORATION4 = True")            
            result = Bool()
            result.data = self.FRONTIER_EXPLORATION
            self.state_publisher_frontier_exploration.publish(result)

    def semantic_map_callback(self, msg):
        # There is no point cloud data. Map is empty.
        # OR NO topic even available. --> ros2 topic list shows no /semantic_pcl
        self.get_logger().info(f"Received: Sematnic Map", once = True)

        if self.FRONTIER_EXPLORATION_ENABLED == False:

            if self.is_point_cloud_zero(msg):
                self.SEMANTIC_MAP_UNKNOWN = True
                self.get_logger().info("SEMANTIC_MAP_UNKNOWN5 = True")
            else:
                self.SEMANTIC_MAP_UNKNOWN = False
                self.get_logger().info("SEMANTIC_MAP_UNKNOWN6 = False")

            result = Bool()
            result.data = self.SEMANTIC_MAP_UNKNOWN
            self.state_publisher_map_unknown.publish(result)
        
        if self.FRONTIER_EXPLORATION_ENABLED == True:

            if self.is_point_cloud_zero(msg):
                self.SEMANTIC_MAP_UNKNOWN = True
                self.get_logger().info("SEMANTIC_MAP_UNKNOWN7 = True")
            else:
                self.SEMANTIC_MAP_UNKNOWN = False
                self.get_logger().info("SEMANTIC_MAP_UNKNOWN8 = False")
                self.FRONTIER_EXPLORATION = True
                result = Bool()
                result.data = self.FRONTIER_EXPLORATION
                self.state_publisher_frontier_exploration.publish(result)

            result = Bool()
            result.data = self.SEMANTIC_MAP_UNKNOWN
            self.state_publisher_map_unknown.publish(result)
        
        

    def random_exploration_callback(self, msg):
        # If there is no data in msg OR all data is 0. x_linear, y_linear, z_linear, w_linear and x_angular ...
        # Robot is not doing random exploration.
        
        if msg.data == False:
            self.FRONTIER_EXPLORATION_ENABLED = True
            self.FRONTIER_EXPLORATION = True

            if (self.FRONTIER_EXPLORATION_ENABLED == True) and (self.SEMANTIC_MAP_UNKNOWN == True):
                self.get_logger().info("SEMANTIC_MAP_UNKNOWN9 = True")
                self.FRONTIER_EXPLORATION = False
            
        else:
            self.FRONTIER_EXPLORATION_ENABLED = False
            self.FRONTIER_EXPLORATION = False
            self.get_logger().info("FRONTIER_EXPLORATION10 = False")
            if (self.FRONTIER_EXPLORATION_ENABLED == False) and (self.SEMANTIC_MAP_UNKNOWN == False):            
                self.get_logger().info("SEMANTIC_MAP_KNOWN2 = True")          
                self.SEMANTIC_MAP_KNOWN = True
                result = Bool()
                result.data = self.SEMANTIC_MAP_KNOWN
                self.state_publisher_map_known.publish(result)
            

        result = Bool()
        result.data = self.FRONTIER_EXPLORATION
        self.state_publisher_frontier_exploration.publish(result)
    

    def is_point_cloud_zero(self, point_cloud_msg):
        # Get the step size (size of a point in bytes)
        point_step = point_cloud_msg.point_step

        # Iterate through the point cloud data
        for i in range(point_cloud_msg.width * point_cloud_msg.height):
            # Calculate the offset for each point
            offset = i * point_step

            # Extract x, y, z values from the point cloud data
            x = struct.unpack_from('f', point_cloud_msg.data, offset)[0]
            y = struct.unpack_from('f', point_cloud_msg.data, offset + 4)[0]
            z = struct.unpack_from('f', point_cloud_msg.data, offset + 8)[0]

            # Check if all x, y, and z values are zero
            if x != 0.0 or y != 0.0 or z != 0.0:
                return False

        # If all points are zero, return True
        return True

def main():
    rclpy.init()

    node = States()

    try:
        print("Waiting for messages. Press Ctrl+C to exit.")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Clean up resources before exiting
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
