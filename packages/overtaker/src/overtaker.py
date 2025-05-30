#!/usr/bin/env python3

import rospy
import math
from duckietown_msgs.msg import FSMState
from std_msgs.msg import Float64
from enum import Enum
from abc import ABC, abstractmethod

OVERTAKER_UPDATE_FREQUENCY = 20.0 # Hz
OVERTAKER_TIMEOUT_DURATION = 5.0 # seconds
OVERTAKING_TURN_1 = math.pi / 4.0 # radians
OVERTAKING_FORWARD_DRIVE_1 = 0.5 # meters
OVERTAKING_TURN_2 = -math.pi / 2.0 # radians
OVERTAKING_FORWARD_DRIVE_2 = 0.5 # meters
OVERTAKING_TURN_3 = math.pi / 4.0 # radians

class OvertakingEvent(Enum):
    START_OVERTAKING = 0
    MOVEMENT_CONTROLLER_SUCCESS = 1
    MOVEMENT_CONTROLLER_FAILURE = 2

class Timer:
    def __init__(self, duration: float):
        self._duration = duration
        self._start_time = None
    def start(self):
        self._start_time = rospy.get_time()
    def is_expired(self) -> bool:
        if self._start_time is None:
            return False
        return (rospy.get_time() - self._start_time) >= self._duration
    def reset(self):
        self._start_time = None

class Overtaker:
    def __init__(self, state: 'OvertakerState',):
        self._state_publisher = rospy.Publisher('/vader/fsm_node/mode', FSMState, queue_size=1)
        self._goal_distance_publisher = rospy.Publisher('/vader/goal_distance', Float64, queue_size=1)
        self._goal_angle_publisher = rospy.Publisher('/vader/goal_angle', Float64, queue_size=1)
        rospy.Subscriber('/vader/fsm_node/mode', FSMState, self.FSM_state_callback, queue_size=1)
        self.transition_to(state)

    def transition_to(self, state: 'OvertakerState'):
        self._state = state
        self._state._context = self
        rospy.loginfo(f"Transitioning to state: {type(state).__name__}")
        self._state.on_enter()  # Call on_enter after context is set

    def on_event(self, event: OvertakingEvent) -> None:
        rospy.loginfo(f"Event received: {event}")
        self._state.on_event(event)

    def update(self):
        self._state.update()

    def FSM_state_callback(self, msg: FSMState):
        if msg.state == 'OVERTAKING_START':
            self.on_event(OvertakingEvent.START_OVERTAKING)
        elif msg.state == 'MOVEMENT_CONTROLLER_SUCCESS':
            self.on_event(OvertakingEvent.MOVEMENT_CONTROLLER_SUCCESS)
        elif msg.state == 'MOVEMENT_CONTROLLER_FAILURE':
            self.on_event(OvertakingEvent.MOVEMENT_CONTROLLER_FAILURE)

    def publish_FSM_state(self, state: str):
        fsm_state_msg = FSMState()
        fsm_state_msg.header.stamp = rospy.Time.now()
        fsm_state_msg.state = state
        self._state_publisher.publish(fsm_state_msg)

    def publish_goal_distance(self, distance: float):
        goal_distance_msg = Float64()
        goal_distance_msg.data = distance
        self._goal_distance_publisher.publish(goal_distance_msg)

    def publish_goal_angle(self, angle: float):
        goal_angle_msg = Float64()
        goal_angle_msg.data = angle
        self._goal_angle_publisher.publish(goal_angle_msg)

class OvertakerState(ABC):
    @property
    def context(self) -> Overtaker:
        return self._context
    
    @context.setter
    def context(self, context: Overtaker) -> None:
        self._context = context

    def on_enter(self):
        pass

    @abstractmethod
    def on_event(self, event: OvertakingEvent) -> None:
        pass

    @abstractmethod
    def update(self) -> None:
        pass

class OvertakerInactiveState(OvertakerState):
    def __init__(self):
        pass

    def on_enter(self):
        pass

    def on_event(self, event: OvertakingEvent) -> None:
        if event == OvertakingEvent.START_OVERTAKING:
            self.context.transition_to(OvertakerTurn1State())

    def update(self) -> None:
        pass

class OvertakerTurn1State(OvertakerState):
    def __init__(self):
        self._timer = Timer(OVERTAKER_TIMEOUT_DURATION)

    def on_enter(self):
        self._timer.start()
        self.context.publish_FSM_state('OVERTAKING_TURN_1')
        self.context.publish_goal_angle(OVERTAKING_TURN_1)

    def on_event(self, event: OvertakingEvent) -> None:
        if event == OvertakingEvent.MOVEMENT_CONTROLLER_SUCCESS:
            self.context.transition_to(OvertakerStraightDrive1State())

    def update(self) -> None:
        if self._timer.is_expired():
            self.context.transition_to(OvertakerTimerExpiredState())

class OvertakerStraightDrive1State(OvertakerState):
    def __init__(self):
        self._timer = Timer(OVERTAKER_TIMEOUT_DURATION)

    def on_enter(self):
        self._timer.start()
        self.context.publish_FSM_state('OVERTAKING_STRAIGHT_DRIVE_1')
        self.context.publish_goal_distance(OVERTAKING_FORWARD_DRIVE_1)

    def on_event(self, event: OvertakingEvent) -> None:
        if event == OvertakingEvent.MOVEMENT_CONTROLLER_SUCCESS:
            self.context.transition_to(OvertakerTurn2State())

    def update(self) -> None:
        if self._timer.is_expired():
            self.context.transition_to(OvertakerTimerExpiredState())

class OvertakerTurn2State(OvertakerState):
    def __init__(self):
        self._timer = Timer(OVERTAKER_TIMEOUT_DURATION)

    def on_enter(self):
        self._timer.start()
        self.context.publish_FSM_state('OVERTAKING_TURN_2')
        self.context.publish_goal_angle(OVERTAKING_TURN_2)

    def on_event(self, event: OvertakingEvent) -> None:
        if event == OvertakingEvent.MOVEMENT_CONTROLLER_SUCCESS:
            self.context.transition_to(OvertakerStraightDrive2State())

    def update(self) -> None:
        if self._timer.is_expired():
            self.context.transition_to(OvertakerTimerExpiredState())

class OvertakerStraightDrive2State(OvertakerState):
    def __init__(self):
        self._timer = Timer(OVERTAKER_TIMEOUT_DURATION)

    def on_enter(self):
        self._timer.start()
        self.context.publish_FSM_state('OVERTAKING_STRAIGHT_DRIVE_2')
        self.context.publish_goal_distance(OVERTAKING_FORWARD_DRIVE_2)

    def on_event(self, event: OvertakingEvent) -> None:
        if event == OvertakingEvent.MOVEMENT_CONTROLLER_SUCCESS:
            self.context.transition_to(OvertakerTurn3State())

    def update(self) -> None:
        if self._timer.is_expired():
            self.context.transition_to(OvertakerTimerExpiredState())

class OvertakerTurn3State(OvertakerState):
    def __init__(self):
        self._timer = Timer(OVERTAKER_TIMEOUT_DURATION)

    def on_enter(self):
        self._timer.start()
        self.context.publish_FSM_state('OVERTAKING_TURN_3')
        self.context.publish_goal_angle(OVERTAKING_TURN_3)

    def on_event(self, event: OvertakingEvent) -> None:
        if event == OvertakingEvent.MOVEMENT_CONTROLLER_SUCCESS:
            self.context.publish_FSM_state('OVERTAKING_SUCCESS')
            self.context.transition_to(OvertakerInactiveState())

    def update(self) -> None:
        if self._timer.is_expired():
            self.context.transition_to(OvertakerTimerExpiredState())

class OvertakerTimerExpiredState(OvertakerState):
    def __init__(self):
        pass

    def on_enter(self):
        self.context.publish_FSM_state('OVERTAKING_FAILURE')
        rospy.logwarn("Overtaking timer expired, transitioning to inactive state.")
        self.context.transition_to(OvertakerInactiveState())

    def on_event(self, event: OvertakingEvent) -> None:
        pass

    def update(self) -> None:
        pass

class OvertakerDriver():
    def __init__(self):
        rospy.init_node('overtaker_node', anonymous=True)
        self._overtaker = Overtaker(OvertakerInactiveState())
        rospy.loginfo("Overtaker node initialized!")

    def run(self):
        rate = rospy.Rate(OVERTAKER_UPDATE_FREQUENCY)
        rospy.loginfo("Overtaker is running.")
        while not rospy.is_shutdown():
            self._overtaker.update()
            rate.sleep()

if __name__ == '__main__':
    try:
        overtaker_driver = OvertakerDriver()
        overtaker_driver.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
