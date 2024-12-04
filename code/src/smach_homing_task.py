#!/usr/bin/env python

import rospy
import smach

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Navigate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['P1', 'P2', 'P3', 'P4', 'navigation_failed'])
        self.point = None
        self.counter = 0
    
    def execute(self, userdata):
        rospy.loginfo('Executing state Navigate')
        if self.counter == 0:
            self.point = 'P2'
        elif self.counter == 1:
            self.point = 'P3'
        elif self.counter == 2:
            self.point = 'P4'
        elif self.counter == 3:
            self.point = 'P1'
        self.counter += 1
        
        rospy.loginfo(f"Navigating to {self.point}")
        
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  
        goal.target_pose.header.stamp = rospy.Time.now()

        if self.point == 'P2':
            goal.target_pose.pose.position.x = 2.0
            goal.target_pose.pose.position.y = 1.0
            goal.target_pose.pose.orientation.w = 1.0
        elif self.point == 'P3':
            goal.target_pose.pose.position.x = 3.0
            goal.target_pose.pose.position.y = 2.0
            goal.target_pose.pose.orientation.w = 1.0
        elif self.point == 'P4':
            goal.target_pose.pose.position.x = 4.0
            goal.target_pose.pose.position.y = 3.0
            goal.target_pose.pose.orientation.w = 1.0
        elif self.point == 'P1':
            goal.target_pose.pose.position.x = 0.0
            goal.target_pose.pose.position.y = 0.0
            goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo(f"Sending goal to {self.point}")
        client.send_goal(goal)

        client.wait_for_result()

        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Successfully navigated to {self.point}")
            return self.point
        else:
            rospy.loginfo(f"Failed to navigate to {self.point} for some reason")
            return 'navigation_failed'

class Find_Pillar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pillar_find', 'pillar_not_find'], ouput_key=['pillar_position'])
        self.pillar_position = None
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Find_Pillar')
        
        # find the pillar
        
        
        # return pillar position
        userdata.pillar_position = self.pillar_poisition
        
class Park_Pillar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['parked', 'parked_failed'], input_keys=['pillar_position'])
        self.pillar_position = None
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Park_Pillar')
        self.pillar_position = userdata.pillar_position
        
        # park the pillar
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map" 
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = self.pillar_position[0]
        goal.target_pose.pose.position.y = self.pillar_position[1]
        goal.target_pose.pose.orientation.w = 1.0  

        rospy.loginfo(f"Sending goal to park at position {self.pillar_position}")
        client.send_goal(goal)

        client.wait_for_result()

        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Successfully parked at the pillar location")
            return 'parked'  
        else:
            rospy.loginfo("Failed to park at the pillar location")
            return 'parked_failed' 
        
def main():
    rospy.init_node('smach_homing_task')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['task_complete', 'task_incomplete'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Navigate', Navigate(), 
                               transitions={'P1':'task_complete',
                                            'P2':'Find_Pillar', 
                                            'P3':'Find_Pillar', 
                                            'P4':'Find_Pillar',
                                            'navigation_failed':'task_incomplete'})
        
        smach.StateMachine.add('Find_Pillar', Find_Pillar(), 
                               transitions={'pillar_find':'Park_Pillar',
                                            'pillar_not_find':'task_incomplete'})
        
        
        smach.StateMachine.add('Park_Pillar', Park_Pillar(), 
                               transitions={'parked_P2':'Navigate',
                                            'parked_P3':'Navigate',
                                            'parked_P4':'Navigate',
                                            'parked_failed':'task_incomplete'})
        

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.loginfo(f"State Machine finished with outcome: {outcome}")


if __name__ == '__main__':
    main()