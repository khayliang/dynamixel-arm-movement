#!/usr/bin/env python3
import rospy
import yaml

from dynamixel_workbench_msgs.srv import DynamixelCommand
from dynamixel_workbench_msgs.msg import DynamixelStateList
from dynamixel_positions.srv import ChangeTorque, MoveToPos, SaveCurrentPos, GetPos

class PositionController:
    def __init__(self):
        self.positions_path = rospy.get_param("/dynamixel_positions/path")

        file_stream = open(self.positions_path, 'r')
        self.positions = yaml.load(file_stream, Loader=yaml.CLoader)
        file_stream.close()

        if self.positions == None:
            self.positions = {}

        self.dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)

        self.torque_state_srv = rospy.Service('/dynamixel_positions/change_torque', ChangeTorque, self.change_torque_state)
        self.move_pos_srv = rospy.Service('/dynamixel_positions/move_to_pos', MoveToPos, self.move_to_position)
        self.save_pos_srv = rospy.Service('/dynamixel_positions/save_current_pos', SaveCurrentPos, self.save_current_pos)
        self.save_pos_srv = rospy.Service('/dynamixel_positions/remove_pos', SaveCurrentPos, self.remove_pos)
        self.get_pos_srv = rospy.Service('/dynamixel_positions/get_pos', GetPos, self.list_positions)
        
        self.torque_enabled = True

    def move_to_position(self, msg):
        position = msg.position
        if position in self.positions:
            servo_pos = self.positions[position]
            if not self.torque_enabled:
                return False
            return self.send_commands_all('Goal_Position', servo_pos)
        else:
            return False

    def change_torque_state(self, msg):
        msg = rospy.wait_for_message('/dynamixel_workbench/dynamixel_state', DynamixelStateList)
        servo_amt = len(msg.dynamixel_state)
        # turn torque off
        if self.torque_enabled:
            self.torque_enabled = False
            return self.send_commands_all('Torque_Enable', [0 for i in range(servo_amt)])
        else:
            self.torque_enabled = True
            return self.send_commands_all('Torque_Enable', [1 for i in range(servo_amt)])

    # value is array 
    def send_commands_all(self, name, values):
        for idx in range(0, len(values)):
            resp = self.dynamixel_command('', idx+1, name, values[idx])
            if resp.comm_result == False:
                return False
        return True
    
    def save_current_pos(self, msg):
        state = rospy.wait_for_message('/dynamixel_workbench/dynamixel_state', DynamixelStateList)
        servo_pos = []
        for servo in state.dynamixel_state:
            servo_pos.append(servo.present_position)
        self.positions[msg.position] = servo_pos

        file_stream = open(self.positions_path, 'w+')
        yaml.dump(self.positions, file_stream, default_flow_style=False, Dumper=yaml.CDumper)
        file_stream.close()

        return True

    def remove_pos(self, msg):
        resp = self.positions.pop(msg.position, None)

        file_stream = open(self.positions_path, 'w+')
        yaml.dump(self.positions, file_stream, default_flow_style=False, Dumper=yaml.CDumper)
        file_stream.close()

        if resp == None:
            return False
        return True


    def list_positions(self, msg):
        return self.positions.keys()

def main():
    '''Initializes and cleanup ros node'''
    dynamixel_positions = PositionController()
    rospy.init_node('dynamixel_positions', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS tracker")    
        

if __name__=="__main__":
    main()

    