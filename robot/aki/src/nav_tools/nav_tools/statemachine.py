import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String  
import subprocess

from nav2_msgs.action import NavigateToPose  
from std_msgs.msg import Empty, Int32
from slam_interface.action import ActivateSlam
from rclpy.executors import MultiThreadedExecutor
import time

class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('statemachine')
        #self._action_client = ActionClient(self, NavigateToPose, 'slam_nav_pid')
        #self._centertable_client = ActionClient(self, NavigateToPose, 'yolo_nav_pid')
        #self._godown_client = ActionClient(self, Godownint, 'godownserver')
        #self._slam_client = ActionClient(self, NavigateToPose, 'rov_loc_nav_pid')
        self._action_client = ActionClient(self, NavigateToPose, 'rov_loc_nav_pid')
        self._centertable_client = ActionClient(self, NavigateToPose, 'yolo_nav_pid')
        self._slam_client = ActionClient(self, ActivateSlam, 'activateslam')
        self.table_pub = self.create_publisher(Int32, 'table_id', 10)

        self.table_id = 1


        self.timer = self.create_timer(2.0, self.timer_callback)
        self.slam_process = None
        self.slam_process1 = None
        self.slam_running = False

        #self.state = 'MissionStart'  # Initial state
        #self.command = 'gototarget'
        # self.state = 'Slam' #test slam activation
        # self.command = 'ActivateSlam' 
        self.state = 'CenterTable' #test yolo activation
        self.command = 'centertable' 




    def timer_callback(self):
        if self.state == 'MissionStart':
            if self.command == 'gototarget':
                self.get_logger().info('started go target')
                table_msg = Int32()
                table_msg.data = self.table_id
                self.table_pub.publish(table_msg)
                #self.centertable()

                self.gototarget()
                self.command = 'wait for gototarget'

            self.get_logger().info(self.state+' '+self.command)

        elif self.state =='CenterTable':
            if self.command=='centertable':
                self.get_logger().info('table gets centered')
                self.command = 'wait for center table'
                self.centertable()
                #self.godown()
            self.get_logger().info(self.state+' '+self.command)

        # elif self.state == 'GoDownTable':
        #     if self.command=='godowntable':
        #         self.get_logger().info('go down for the table')
        
        elif self.state == 'Slam':
            if self.command == 'ActivateSlam':
                self.get_logger().info('starting to feed')
                self.command = 'wait for table gets fed'
                if not self.slam_running:
                    self.get_logger().info('starting SLAM...')
                    self.slam_process1 = subprocess.Popen(['ros2', 'run', 'ros2_orb_slam3', 'mono_driver_node.py', '--ros-args -p setting_name:=EuRoC -p image_seq:=sample_euroc_MH05'])
                    time.sleep(0.1)
                    self.slam_process = subprocess.Popen(['ros2', 'run', 'ros2_orb_slam3','mono_node_cpp', '--ros-args -p node_name_arg:=mono_slam_cpp']) # SLAM should alwyzs be active
                    self.slam_running = True

                self.slam()
            self.get_logger().info(self.state+' '+self.command)

        elif self.state == 'Finished':
            if self.command == 'terminate':
                self.get_logger().info('Mission Completed')
                if self.slam_running:
                    self.slam_process.terminate()
                    self.slam_process.wait()
                    self.slam_process1.terminate()
                    self.slam_process1.wait()
                    self.slam_running = False
                    self.get_logger().info('slam terminated')
            self.get_logger().info(self.state+' '+self.command)

    def slam(self):
        goal_msga = ActivateSlam.Goal()
        self._slam_client.wait_for_server()
        goal_msga.activate = True
        self._send_goal_future = self._slam_client.send_goal_async(goal_msga)
        self._send_goal_future.add_done_callback(self.slam_init_callback)

    def slam_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.success))
        # Access the first node's parameter to increment table_id
        if self.slam_running:
            self.slam_process.terminate()
            self.slam_process.wait()
            self.slam_process1.terminate()
            self.slam_process1.wait()
            self.slam_running = False
            self.get_logger().info('slam terminated')
        self.table_id += 1
        table_msg = Int32()
        table_msg.data = self.table_id
        self.table_pub.publish(table_msg)
        self.state = 'MissionStart'  # Finally, here we want to switch to gototarget -> next table
        self.command = 'gototarget' #go up first
        self.get_logger().info('Completed Feeding process')
    
    def slam_init_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('slam crashed :(')
            return

        self.get_logger().info('started slam')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.slam_callback)

    


    # def godown(self):
    #     goal_msga = Godownint.Goal()
    #     goal_msga.targetx = 1.0
    #     goal_msga.targety = 0.0
    #     self._godown_client.wait_for_server()
    #     self._send_goal_future = self._godown_client.send_goal_async(goal_msga)
    #     self._send_goal_future.add_done_callback(self.godown_init_callback)

    # def godown_callback(self, future):
    #     result = future.result().result
    #     self.get_logger().info('Result: {0}'.format(result.success))
    #     self.state = 'Finished'  # Initial state
    #     self.command = 'aaa'
    #     self.get_logger().info('Completed Task Go Down')

    # def godown_init_callback(self, future):
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().info('go target rejected :(')
    #         return

    #     self.get_logger().info('started go target')

    #     self._get_result_future = goal_handle.get_result_async()
    #     self._get_result_future.add_done_callback(self.godown_callback)



    def centertable(self):
        goal_msga = NavigateToPose.Goal()
        self._centertable_client.wait_for_server()
        self._send_goal_future = self._centertable_client.send_goal_async(goal_msga)
        self._send_goal_future.add_done_callback(self.centertable_init_callback)

    def centertable_callback(self, future):
        result = future.result()
        #self.get_logger().info('Result: {0}'.format(result.success))
        if result.status == 3:
            self.get_logger().info('Result: success')
        """self.state = 'GoDownTable'  # Initial state
        self.command = 'godowntable'"""
        self.state = 'Slam'  # Keep in mind that we are now 1 meter above the table, godown is still necessary
        self.command = 'ActivateSlam'
        self.get_logger().info('Completed Task Center Table')

    def centertable_init_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('center table rejected :(')
            return

        self.get_logger().info('started center table')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.centertable_callback)


    def gototarget(self):
        goal_msg = NavigateToPose.Goal()

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.gototarget_init_callback)

    def gototarget_callback(self, future):
        result = future.result()
        #self.get_logger().info('Result: {0}'.format(result.success))
        if result.status == 3:
            self.get_logger().info('Result: success')
        self.state = 'CenterTable'  # also include moving down later on
        self.command = 'centertable'

        self.get_logger().info('Completed Task Mission Start')

        
    def gototarget_init_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('go target rejected :(')
            return

        self.get_logger().info('started go target')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.gototarget_callback)
        
    # def gototarget_feedback(self, feedback_msg):
    #     self.get_logger().info(f'[RovLocNav] {feedback_msg.feedback}')







def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    #rclpy.spin(action_client)
    
    
    executor = MultiThreadedExecutor()
    executor.add_node(action_client)

    try:
        executor.spin()
    except KeyboardInterrupt:
        action_client.slam_process.terminate()
        action_client.slam_process.wait()
        action_client.slam_process1.terminate()
        action_client.slam_process1.wait()
        pass
    finally:
        action_client.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()