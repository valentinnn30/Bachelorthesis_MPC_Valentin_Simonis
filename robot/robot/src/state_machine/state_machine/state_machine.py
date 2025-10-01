import rclpy
from rclpy.node import Node 

from std_msgs.msg import String


class RobotStateMachine(Node): 
    def __init__(self):
        super().__init__('robot_fsm') 
        self.state = 'MissonStart' # Subscribers and publishers for robot's actions 
        self.cmd_sub = self.create_subscription(String, '/robot_command', self.cmd_callback, 10) 
        self.state_pub = self.create_publisher(String, '/robot_state', 10) # Timer to publish the current state 
        self.timer = self.create_timer(0.5, self.publish_state) 
        
        self.x = 1.0
        self.y = 0.0
        
    def cmd_callback(self, msg): 
        command = msg.data 
        self.get_logger().info(f'Received command: {command}') 
        #good place, to do some error handling here
        #the commands for these errors should be specified somewhere else

        if self.state == 'MissonStart' and command == 'table_detected': 
            self.state = 'MoveToTable' 
            self.start_to_table() 

        elif self.state == 'MoveToTable' and command == 'arrived_table': 
            #arrived at table is also arrived at coral in this case
            #can be seperated, that we first find the center and then the top left corner etc
            self.state = 'Feeding' 
            self.start_feeding() 

        
        elif self.state == 'Feeding' and command == 'coral_fed': 
            #feeding of one group of corals, then moving to the next group
            self.state = 'MoveToCoral' 
            self.start_table_movement() 
        
        elif self.state == 'MoveToCoral' and command == 'arrived_coral': 
            self.state = 'Feeding' 
            self.start_feeding()

        elif self.state == 'Feeding' and command == 'table_fed':
            #not only one group of corals but all corals on the table are fed now
            self.state = 'MoveUp' 
            self.start_upmovement()

        elif self.state == 'MoveUp' and command == 'new_table_detected':
            self.state = 'MoveToTable' 
            self.start_navigation()

        elif self.state == 'MoveUp' and command == 'all_table_visited':
            #no new table detected, since all are already visited & fed
            self.state = 'GoHome' 
            self.end_task()
    
    def start_navigation(self): 
        self.get_logger().info('Starting navigation...') 
        # Logic to start navigation to a location 
    def start_to_table(self):
        self.get_logger().info('Start movint towards table...') 
        
    def start_feeding(self): 
        self.get_logger().info('Feeding corals...') 
        # Logic to perform the task at the destination 
    
    def start_table_movement(self): 
        self.get_logger().info('Moving to corals...') 
        # Logic to start navigation to next coral

    def start_upmovement(self): 
        self.get_logger().info('Moving up now...') 
        # Logic to start moving up
        
    def end_task(self): 
        self.get_logger().info('Task complete. Returning to idle.') 
        
    def publish_state(self): 
        state_msg = String() 
        state_msg.data = self.state 
        self.state_pub.publish(state_msg) 
        self.get_logger().info(f'Current State: {self.state}') 
        
        
def main(args=None): 
    rclpy.init(args=args) 
    robot_fsm = RobotStateMachine() 
    rclpy.spin(robot_fsm) 
    robot_fsm.destroy_node() 
    rclpy.shutdown()