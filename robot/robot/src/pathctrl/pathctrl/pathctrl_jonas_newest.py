import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from move_interface.action import Move
from gototable_interface.action import GoTable
from godown_interface.action import Godownint
from slam_interface.action import ActivateSlam


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Move, 'navigatetable')
        self._centertable_client = ActionClient(self, GoTable, 'gototable')
        #self._godown_client = ActionClient(self, Godownint, 'godownserver')
        self._slam_client = ActionClient(self, ActivateSlam, 'activateslam')


        self.timer = self.create_timer(2.0, self.timer_callback)

        self.state = 'MissionStart'  # Initial state
        self.command = 'gototarget'

    def timer_callback(self):
        if self.state == 'MissionStart':
            if self.command == 'gototarget':
                self.get_logger().info('started go target')
                #self.centertable()

                self.gototarget()
                self.command = 'wait for gototarger'

            self.get_logger().info(self.state+' '+self.command)

        elif self.state =='CenterTable':
            if self.command=='centertable':
                self.get_logger().info('table gets centeres')
                self.command = 'wait for center table'
                #self.godown()
            self.get_logger().info(self.state+' '+self.command)

        # elif self.state == 'GoDownTable':
        #     if self.command=='godowntable':
        #         self.get_logger().info('go down for the table')
        
        elif self.state == 'Slam':
            if self.command == 'ActivateSlam':
                self.get_logger().info('starting to feed')
                self.slam()

    def slam(self):
        goal_msga = ActivateSlam.Goal()
        self._slam_client.wait_for_server()
        goal_msga.activate = True
        self._send_goal_future = self._slam_client.send_goal_async(goal_msga)
        self._send_goal_future.add_done_callback(self.slam_init_callback)

    def slam_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.success))
        self.state = 'Finished'  # Initial state
        self.command = 'aaa'
        self.get_logger().info('slam started')
    
    def slam_init_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('go target rejected :(')
            return

        self.get_logger().info('started go target')

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
        goal_msga = GoTable.Goal()
        goal_msga.targetx = 1.0
        goal_msga.targety = 0.0
        self._centertable_client.wait_for_server()
        self._send_goal_future = self._centertable_client.send_goal_async(goal_msga)
        self._send_goal_future.add_done_callback(self.centertable_init_callback)

    def centertable_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.success))
        self.state = 'GoDownTable'  # Initial state
        self.command = 'godowntable'
        self.get_logger().info('Completed Task Center Table')

    def centertable_init_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('go target rejected :(')
            return

        self.get_logger().info('started center table')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.centertable_callback)


    def gototarget(self):
        goal_msg = Move.Goal()
        goal_msg.targetx = 1.0
        goal_msg.targety = 1.0

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.gototarget_init_callback)

    def gototarget_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.success))
        self.state = 'CenterTable'  # Initial state
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






def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()