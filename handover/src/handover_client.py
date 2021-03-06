#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
import sys

from smach_tutorials.msg import TestAction, TestGoal
from actionlib import *
from actionlib_msgs.msg import *

def static_passive():
    rospy.init_node('handover_client')

    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted', 'End'])

    with sm0:
        smach.StateMachine.add('Init',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=0)),
                               {'succeeded':'Detect','aborted':'Init'})

        smach.StateMachine.add('Detect',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=1)),
                               {'succeeded':'Move_to','aborted':'Detect'})

        smach.StateMachine.add('Move_to',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=2)),
                               {'succeeded':'Grasp_back','aborted':'Move_to'})

        smach.StateMachine.add('Grasp_back',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=3)),
                               {'succeeded':'End','aborted':'Grasp_back'})

    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm0, '/SM_ROOT')
    sis.start()

    outcome = sm0.execute()

    rospy.spin()
    sis.stop()

def static_active():
    rospy.init_node('handover_client')

    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted', 'End'])

    with sm0:
        smach.StateMachine.add('Init',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=0)),
                               {'succeeded':'Detect','aborted':'Init'})

        smach.StateMachine.add('Detect',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=1)),
                               {'succeeded':'Move_to','aborted':'Detect'})

        smach.StateMachine.add('Move_to',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=2)),
                               {'succeeded':'Wait_object','aborted':'Move_to'})

        smach.StateMachine.add('Wait_object',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=5)),
                               {'succeeded':'Grasp_back','aborted':'Wait_object'})

        smach.StateMachine.add('Grasp_back',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=3)),
                               {'succeeded':'End','aborted':'Grasp_back'})

    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm0, '/SM_ROOT')
    sis.start()

    outcome = sm0.execute()

    rospy.spin()
    sis.stop()

def dynamic_passive():
    rospy.init_node('handover_client')

    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted', 'End'])

    with sm0:
        smach.StateMachine.add('Init',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=0)),
                               {'succeeded':'Detect','aborted':'Init'})

        smach.StateMachine.add('Detect',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=1)),
                               {'succeeded':'Move_to','aborted':'Detect'})

        smach.StateMachine.add('Move_to',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=2)),
                               {'succeeded':'Check_dis','aborted':'Move_to'})

        smach.StateMachine.add('Check_dis',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=4)),
                               {'succeeded':'Grasp_back','aborted':'Detect'})

        smach.StateMachine.add('Grasp_back',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=3)),
                               {'succeeded':'End','aborted':'Grasp_back'})

    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm0, '/SM_ROOT')
    sis.start()

    outcome = sm0.execute()

    rospy.spin()
    sis.stop()

def dynamic_active():
    rospy.init_node('handover_client')

    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted', 'End'])

    with sm0:
        smach.StateMachine.add('Init',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=0)),
                               {'succeeded':'Detect','aborted':'Init'})

        smach.StateMachine.add('Detect',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=1)),
                               {'succeeded':'Move_to','aborted':'Detect'})

        smach.StateMachine.add('Move_to',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=2)),
                               {'succeeded':'Check_dis','aborted':'Move_to'})

        smach.StateMachine.add('Check_dis',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=4)),
                               {'succeeded':'Wait_object','aborted':'Detect'})

        smach.StateMachine.add('Wait_object',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=5)),
                               {'succeeded':'Grasp_back','aborted':'Wait_object'})

        smach.StateMachine.add('Grasp_back',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=3)),
                               {'succeeded':'End','aborted':'Grasp_back'})

    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm0, '/SM_ROOT')
    sis.start()

    outcome = sm0.execute()

    rospy.spin()
    sis.stop()

def hybird():
    rospy.init_node('handover_client')

    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted', 'End'])

    with sm0:
        smach.StateMachine.add('Init',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=0)),
                               {'succeeded':'Detect and Go target','aborted':'Init'})

        smach.StateMachine.add('Detect and Go target',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=1)),
                               {'succeeded':'Check_dis','aborted':'Detect and Go target'})

        smach.StateMachine.add('Check_dis',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=3)),
                               {'succeeded':'Wait_object','aborted':'Detect and Go target'})

        smach.StateMachine.add('Wait_object',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=4)),
                               {'succeeded':'Grasp_back','aborted':'Wait_object'})

        smach.StateMachine.add('Grasp_back',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=2)),
                               {'succeeded':'End','aborted':'Grasp_back'})

    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm0, '/SM_ROOT')
    sis.start()

    outcome = sm0.execute()

    rospy.spin()
    sis.stop()
    # rospy.on_shutdown()

def multi_view():
    rospy.init_node('handover_client')

    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted', 'End'])

    with sm0:
        smach.StateMachine.add('Init',
                               smach_ros.SimpleActionState('multi_handover_action', TestAction,
                               goal = TestGoal(goal=0)),
                               {'succeeded':'Detect','aborted':'Init'})

        smach.StateMachine.add('Detect',
                               smach_ros.SimpleActionState('multi_handover_action', TestAction,
                               goal = TestGoal(goal=1)),
                               {'succeeded':'Move_to','aborted':'Detect'})

        smach.StateMachine.add('Move_to',
                               smach_ros.SimpleActionState('multi_handover_action', TestAction,
                               goal = TestGoal(goal=2)),
                               {'succeeded':'Grasp_back','aborted':'Move_to'})

        smach.StateMachine.add('Grasp_back',
                               smach_ros.SimpleActionState('multi_handover_action', TestAction,
                               goal = TestGoal(goal=3)),
                               {'succeeded':'End','aborted':'Grasp_back'})

    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm0, '/SM_ROOT')
    sis.start()

    outcome = sm0.execute()

    rospy.spin()
    sis.stop()

def open_cap():
    rospy.init_node('handover_client')

    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted', 'End'])

    with sm0:
        smach.StateMachine.add('Init',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=0)),
                               {'succeeded':'Detect and Go target','aborted':'Init'})

        smach.StateMachine.add('Detect and Go target',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=1)),
                               {'succeeded':'to_parallel','aborted':'Detect and Go target'})

        smach.StateMachine.add('to_parallel',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=5)),
                               {'succeeded':'Open_cap_1','aborted':'to_parallel'})

        smach.StateMachine.add('Open_cap_1',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=6)),
                               {'succeeded':'to_parallel_2','aborted':'Open_cap_1'})

        smach.StateMachine.add('to_parallel_2',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=5)),
                               {'succeeded':'Open_cap_2','aborted':'to_parallel_2'})

        smach.StateMachine.add('Open_cap_2',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=6)),
                               {'succeeded':'Init2','aborted':'to_parallel_2'})

        smach.StateMachine.add('Init2',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=0)),
                               {'succeeded':'End','aborted':'Init2'})

    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm0, '/SM_ROOT')
    sis.start()

    outcome = sm0.execute()

    rospy.spin()
    sis.stop()

def hand_off():
    rospy.init_node('handover_client')

    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted', 'End'])

    with sm0:
        smach.StateMachine.add('Init',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=0)),
                               {'succeeded':'Wait_object','aborted':'Init'})

        smach.StateMachine.add('Wait_object',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=4)),
                               {'succeeded':'Grasp_back','aborted':'Wait_object'})


        smach.StateMachine.add('Grasp_back',
                               smach_ros.SimpleActionState('handover_action', TestAction,
                               goal = TestGoal(goal=2)),
                               {'succeeded':'End','aborted':'Grasp_back'})

    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm0, '/SM_ROOT')
    sis.start()

    outcome = sm0.execute()

    rospy.spin()
    sis.stop()
    
if __name__ == '__main__':
    print("Chose strategies : Handover 1 / Multi_view Handover 2 / Open cover 3 / Hand Off 4")
    mode = int(input("Enter : "))
    while True:
        if mode == 1:
            hybird()
            break
        elif mode == 2:
            multi_view()
            break
        elif mode == 3:
            open_cap()
            break
        elif mode == 4:
            hand_off()
            break
        else:
            mode = int(input("Re Enter : "))


