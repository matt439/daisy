#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, AprilTagDetectionArray
from std_msgs.msg import Int8
from enum import Enum
from abc import ABC, abstractmethod

# Autopilot node constants
AUTOPILOT_UPDATE_FREQUENCY = 20  # Hz

# FSM states
LANE_FOLLOWING_FSM_STATE = "LANE_FOLLOWING"
NORMAL_JOYSTICK_CONTROL_FSM_STATE = "NORMAL_JOYSTICK_CONTROL"

OVERTAKING_START_FSM_STATE = 'OVERTAKING_START'
OVERTAKING_SUCCESS_FSM_STATE = 'OVERTAKING_SUCCESS'
OVERTAKING_FAILURE_FSM_STATE = 'OVERTAKING_FAILURE'

TURNING_START_FSM_STATE = 'TURNING_START'
TURNING_SUCCESS_FSM_STATE = 'TURNING_SUCCESS'
TURNING_FAILURE_FSM_STATE = 'TURNING_FAILURE'

STOPPING_START_FSM_STATE = 'STOPPING_START'
STOPPING_SUCCESS_FSM_STATE = 'STOPPING_SUCCESS'
STOPPING_FAILURE_FSM_STATE = 'STOPPING_FAILURE'

APPROACHING_SIGN_START_FSM_STATE = 'APPROACHING_SIGN_START'
APPROACHING_SIGN_SUCCESS_FSM_STATE = 'APPROACHING_SIGN_SUCCESS'
APPROACHING_SIGN_FAILURE_FSM_STATE = 'APPROACHING_SIGN_FAILURE'

# Stop sign constants
STOP_SIGN_WAITING_TIME = 5.0  # seconds
LANE_FOLLOWING_STOP_SIGN_TIME = 3.0  # seconds
STOP_SIGN_IDS = [1, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38]

# Overtaking constants
OVERTAKING_TIMEOUT_DURATION = 20.0  # seconds
CAR_WAITING_TIME = 5.0  # seconds

# Intersection constants
LEFT_INTERSECTION_SIGNS_IDS = [10, 61, 62, 63, 64]
RIGHT_INTERSECTION_SIGNS_IDS = [9, 57, 58, 59, 60]
T_INTERSECTION_SIGNS_IDS = [11, 65, 66, 67, 68]
SIGN_WAITING_DURATION = 5.0  # seconds
TURNING_TIMEOUT_DURATION = 10.0  # seconds

# Lane controller node parameters constants
LANE_CONTROLLER_NODE_V_BAR = "/vader/lane_controller_node/v_bar" # nominal velocity in m/s
LANE_CONTROLLER_NODE_K_D = "/vader/lane_controller_node/k_d" # proportional term for lateral deviation
LANE_CONTROLLER_NODE_K_THETA = "/vader/lane_controller_node/k_theta" # proportional term for heading deviation
LANE_CONTROLLER_NODE_K_ID = "/vader/lane_controller_node/k_Id" # integral term for lateral deviation
LANE_CONTROLLER_NODE_K_IPHI = "/vader/lane_controller_node/k_IphI" # integral term for lateral deviation?
LANE_CONTROLLER_NODE_THETA_THRES_MIN = "/vader/lane_controller_node/theta_thres_min" # minimum value for heading error
LANE_CONTROLLER_NODE_THETA_THRES_MAX = "/vader/lane_controller_node/theta_thres_max" # maximum value for heading error
V_BAR = 0.3 # clamped from 0 to 5, default 0.19
K_D = -30.0 # clamped from -100 to 100, default -6.0
K_THETA = -9.0 # clamped from -100 to 100, default -5.0
K_ID = -0.3 # clamped from -100 to 100, default -0.3
K_IPHI = 0.0 # clamped from -100 to 100, default 0.0
THETA_THRES_MIN = -0.5 # clamped from -100 to 100, default -0.5
THETA_THRES_MAX = 0.75 # clamped from -100 to 100, default 0.75

# Lane controller node parameters which cannot be set dynamically:
#
# d_thres (:obj:`float`): Maximum value for lateral error
# d_offset (:obj:`float`): Goal offset from center of the lane
# integral_bounds (:obj:`dict`): Bounds for integral term
# d_resolution (:obj:`float`): Resolution of lateral position estimate
# phi_resolution (:obj:`float`): Resolution of heading estimate
# omega_ff (:obj:`float`): Feedforward part of controller
# verbose (:obj:`bool`): Verbosity level (0,1,2)
# stop_line_slowdown (:obj:`dict`): Start and end distances for slowdown at stop lines

class DuckieBotEvent(Enum):
    STOP_SIGN_DETECTED = 0
    BOT_BECOMES_STOPPED = 1
    CAR_DETECTED = 2
    CAR_REMOVED = 3
    TURN_LEFT_SIGN_DETECTED = 4
    TURN_RIGHT_SIGN_DETECTED = 5
    STOPPING_SUCCESS = 6
    STOPPING_FAILURE = 7
    OVERTAKING_SUCCESS = 8
    OVERTAKING_FAILURE = 9
    TURNING_SUCCESS = 10
    TURNING_FAILURE = 11
    PAUSE_COMMAND_RECEIVED = 12
    RESUME_COMMAND_RECEIVED = 13
    APPROACHING_SIGN_SUCCESS = 14
    APPROACHING_SIGN_FAILURE = 15

class Direction (Enum):
    LEFT = 1
    RIGHT = 2

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

class Duckiebot():
    def __init__(self, state: 'DuckiebotState', state_pub, stopping_pub,
                 overtaking_pub, turning_pub, approaching_sign_pub) -> None:
        self._state_publisher = state_pub
        self._stopping_publisher = stopping_pub
        self._overtaking_publisher = overtaking_pub
        self._turning_publisher = turning_pub
        self._approaching_sign_publisher = approaching_sign_pub
        self._sign_tag_id = None
        rospy.loginfo("Duckiebot class initialized!")
        self.transition_to(state)

    def transition_to(self, state: 'DuckiebotState'):
        self._state = state
        self._state._context = self
        rospy.loginfo(f"Transitioning to state: {type(state).__name__}")
        self._state.on_enter()  # Call on_enter after context is set

    def on_event(self, event: DuckieBotEvent) -> None:
        rospy.loginfo(f"Event received: {event}")
        self._state.on_event(event)

    def update(self):
        self._state.update()

    def publish_FSM_state(self, state: str):
        fsm_state_msg = FSMState()
        fsm_state_msg.header.stamp = rospy.Time.now()
        fsm_state_msg.state = state
        self._state_publisher.publish(fsm_state_msg)

    def publish_stopping_goal(self):
        stopping_goal_msg = Int8()
        stopping_goal_msg.data = 1  # 1 indicates the bot should stop
        self._stopping_publisher.publish(stopping_goal_msg)

    def stop_bot(self):
        self.publish_stopping_goal()

    def publish_overtaking_goal(self):
        overtaking_goal_msg = Int8()
        overtaking_goal_msg.data = 1  # 1 indicates the bot should start overtaking
        self._overtaking_publisher.publish(overtaking_goal_msg)

    def publish_turning_goal(self, direction: Direction):
        turning_goal_msg = Int8()
        if direction == Direction.LEFT:
            turning_goal_msg.data = 1  # 1 indicates the bot should turn left
        elif direction == Direction.RIGHT:
            turning_goal_msg.data = 2  # 2 indicates the bot should turn right
        else:
            rospy.logerr("Invalid direction for turning goal.")
            return
        self._turning_publisher.publish(turning_goal_msg)

    def publish_approaching_sign_goal(self, sign_type: int):
        approaching_sign_goal_msg = Int8()
        approaching_sign_goal_msg.data = sign_type
        self._approaching_sign_publisher.publish(approaching_sign_goal_msg)

    def get_sign_tag_id(self) -> int:
        return self._sign_tag_id

class DuckiebotState(ABC):
    @property
    def context(self) -> Duckiebot:
        return self._context
    
    @context.setter
    def context(self, context: Duckiebot) -> None:
        self._context = context

    @abstractmethod
    def on_enter(self) -> None:
        pass

    @abstractmethod
    def on_event(self, event: DuckieBotEvent) -> None:
        pass

    @abstractmethod
    def update(self) -> None:
        pass

class PauseState(DuckiebotState):
    def __init__(self):
        pass

    def on_enter(self):
        self._context.publish_FSM_state('PAUSED')

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.RESUME_COMMAND_RECEIVED:
            self.context.transition_to(LaneFollowingState())

    def update(self) -> None:
        pass

class LaneFollowingState(DuckiebotState):
    def __init__(self):
        pass

    def on_enter(self):
        self._context.publish_FSM_state(LANE_FOLLOWING_FSM_STATE)

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.PAUSE_COMMAND_RECEIVED:
            self.context.transition_to(PauseState())
        elif event == DuckieBotEvent.STOP_SIGN_DETECTED:
            self.context.transition_to(StoppingForStopSignState())
        elif event == DuckieBotEvent.CAR_DETECTED:
            self.context.transition_to(StoppingForCarState())
        elif event == DuckieBotEvent.TURN_LEFT_SIGN_DETECTED:
            self.context.transition_to(ApproachingTurnLeftSignState(self._context.get_sign_tag_id()))
        elif event == DuckieBotEvent.TURN_RIGHT_SIGN_DETECTED:
            self.context.transition_to(ApproachingTurnRightSignState(self._context.get_sign_tag_id()))

    def update(self) -> None:
        pass

class StoppingForStopSignState(DuckiebotState):
    def __init__(self):
        pass

    def on_enter(self):
        self._context.publish_FSM_state(NORMAL_JOYSTICK_CONTROL_FSM_STATE) # Stop lane following
        self._context.stop_bot() # Stop the robot

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.PAUSE_COMMAND_RECEIVED:
            self.context.transition_to(PauseState())
        elif event == DuckieBotEvent.BOT_BECOMES_STOPPED:
            self.context.transition_to(WaitingAtStopSignState())

    def update(self) -> None:
        pass

class WaitingAtStopSignState(DuckiebotState):
    def __init__(self):
        self._timer = Timer(STOP_SIGN_WAITING_TIME)

    def on_enter(self):
        self._timer.start()

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.PAUSE_COMMAND_RECEIVED:
            self.context.transition_to(PauseState())

    def update(self) -> None:
        if self._timer.is_expired():
            self.context.transition_to(LaneFollowingStopSignState())

class LaneFollowingStopSignState(DuckiebotState):
    def __init__(self):
        self._timer = Timer(LANE_FOLLOWING_STOP_SIGN_TIME)

    def on_enter(self):
        self._context.publish_FSM_state(LANE_FOLLOWING_FSM_STATE)
        self._timer.start()

    # No event handling for stop sign detection in this state
    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.PAUSE_COMMAND_RECEIVED:
            self.context.transition_to(PauseState())
        elif event == DuckieBotEvent.CAR_DETECTED:
            self.context.transition_to(StoppingForCarState())
        elif event == DuckieBotEvent.TURN_LEFT_SIGN_DETECTED:
            self.context.transition_to(ApproachingTurnLeftSignState(self._context.get_sign_tag_id()))
        elif event == DuckieBotEvent.TURN_RIGHT_SIGN_DETECTED:
            self.context.transition_to(ApproachingTurnRightSignState(self._context.get_sign_tag_id()))

    def update(self) -> None:
        if self._timer.is_expired():
            self.context.transition_to(LaneFollowingState())
    
class StoppingForCarState(DuckiebotState):
    def __init__(self):
        pass

    def on_enter(self):
        self._context.publish_FSM_state(NORMAL_JOYSTICK_CONTROL_FSM_STATE) # Stop lane following
        self._context.stop_bot() # Stop the robot

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.PAUSE_COMMAND_RECEIVED:
            self.context.transition_to(PauseState())
        elif event == DuckieBotEvent.BOT_BECOMES_STOPPED:
            self.context.transition_to(WaitingForCarState())

    def update(self) -> None:
        pass

class WaitingForCarState(DuckiebotState):
    def __init__(self):
        self._timer = Timer(CAR_WAITING_TIME)

    def on_enter(self):
        self._timer.start()

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.PAUSE_COMMAND_RECEIVED:
            self.context.transition_to(PauseState())
        elif event == DuckieBotEvent.CAR_REMOVED:
            self.context.transition_to(LaneFollowingState())

    def update(self) -> None:
        if self._timer.is_expired():
            self.context.transition_to(OvertakingState())

class OvertakingState(DuckiebotState):
    def __init__(self):
        self._timer = Timer(OVERTAKING_TIMEOUT_DURATION)
    
    def on_enter(self):
        self._timer.start()
        # Send command to the overtaker node to start overtaking
        self._context.publish_overtaking_goal()

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.PAUSE_COMMAND_RECEIVED:
            self.context.transition_to(PauseState())
        elif event == DuckieBotEvent.OVERTAKING_SUCCESS:
            self.context.transition_to(LaneFollowingState())
        elif event == DuckieBotEvent.OVERTAKING_FAILURE:
            rospy.logerr("Overtaking failed. Transitioning to lane following state.")
            self.context.transition_to(LaneFollowingState())

    def update(self) -> None:
        if self._timer.is_expired():
            rospy.logwarn("Overtaking timer expired, transitioning to lane following state.")
            self.context.transition_to(LaneFollowingState())

class TurningLeftState(DuckiebotState):
    def __init__(self, tag_id: int):
        self._tag_id = tag_id
        self._timer = Timer(TURNING_TIMEOUT_DURATION)

    def on_enter(self):
        self._timer.start()
        self._context.publish_turning_goal(Direction.LEFT)

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.PAUSE_COMMAND_RECEIVED:
            self.context.transition_to(PauseState())
        elif event == DuckieBotEvent.TURNING_SUCCESS:
            self.context.transition_to(LaneFollowingState())
        elif event == DuckieBotEvent.TURNING_FAILURE:
            rospy.logerr("Turning left failed. Transitioning to lane following state.")
            self.context.transition_to(LaneFollowingState())

    def update(self) -> None:
        if self._timer.is_expired():
            rospy.logwarn("Turning left timer expired, transitioning to lane following state.")
            self.context.transition_to(LaneFollowingState())

class TurningRightState(DuckiebotState):
    pass

class WaitingAtTurnLeftSignState(DuckiebotState):
    def __init__(self, tag_id: int):
        self._tag_id = tag_id
        self._timer = Timer(SIGN_WAITING_DURATION)
    
    def on_enter(self):
        self._timer.start()

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.PAUSE_COMMAND_RECEIVED:
            self.context.transition_to(PauseState())
    
    def update(self) -> None:
        if self._timer.is_expired():
            self.context.transition_to(TurningLeftState(self._tag_id))

class WaitingAtTurnRightSignState(DuckiebotState):
    pass

class ApproachingTurnLeftSignState(DuckiebotState):
    def __init__(self, tag_id: int):
        self._tag_id = tag_id

    def on_enter(self):
        self._context.publish_approaching_sign_goal(self._tag_id)

    def on_event(self, event: DuckieBotEvent) -> None:
        if event == DuckieBotEvent.PAUSE_COMMAND_RECEIVED:
            self.context.transition_to(PauseState())
        elif event == DuckieBotEvent.APPROACHING_SIGN_SUCCESS:
            self.context.transition_to(WaitingAtTurnLeftSignState())
        elif event == DuckieBotEvent.APPROACHING_SIGN_FAILURE:
            rospy.logerr("Approaching left turn sign failed. Transitioning to lane following state.")
            self.context.transition_to(LaneFollowingState())

class ApproachingTurnRightSignState(DuckiebotState):
    pass

class Autopilot:
    def __init__(self):
        #Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)

        self._sign_tag_id = None
        
        self.cmd_vel_pub = rospy.Publisher('/vader/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self._state_publisher = rospy.Publisher('/vader/fsm_node/mode', FSMState, queue_size=1)
        self._stopping_publisher = rospy.Publisher('/vader/movement_controller_node/goal_stopping', Int8, queue_size=1)
        self._overtaking_publisher = rospy.Publisher('/vader/movement_controller_node/goal_overtaking', Int8, queue_size=1)
        self._turning_publisher = rospy.Publisher('/vader/movement_controller_node/goal_turning', Int8, queue_size=1)
        self._approaching_sign_publisher = rospy.Publisher('/vader/movement_controller_node/goal_approaching_sign', Int8, queue_size=1)
        rospy.Subscriber('/vader/fsm_node/mode', FSMState, self.FSM_state_callback, queue_size=1)
        rospy.Subscriber('/vader/apriltag_detector_node/detections',
                         AprilTagDetectionArray, self.april_tag_callback, queue_size=1)
        rospy.Subscriber('/vader/obstacle_detector', Int8, self.obstacle_callback, queue_size=1)
        rospy.Subscriber('/vader/autopilot_node/mode', Int8, self.autopilot_control_callback, queue_size=1)

        self.set_lane_following_parameters()

        self._duckiebot = Duckiebot(LaneFollowingState(),
                                    self._state_publisher,
                                    self._stopping_publisher,
                                    self._overtaking_publisher,
                                    self._turning_publisher,
                                    self._approaching_sign_publisher)

        rospy.loginfo("Initialized autopilot node!")
 
    def autopilot_control_callback(self, msg: Int8):
        if msg.data != 0 and msg.data != 1:
            rospy.logwarn("Unknown autopilot control state received.")
            return
        
        if msg.data == 1:
            self._duckiebot.on_event(DuckieBotEvent.RESUME_COMMAND_RECEIVED)
        else: # msg.data == 0
            self._duckiebot.on_event(DuckieBotEvent.PAUSE_COMMAND_RECEIVED)

    def obstacle_callback(self, msg: Int8):
        if msg.data != 0 and msg.data != 1:
            rospy.logwarn("Unknown obstacle state received.")
            return

        if msg.data == 1:
            self._duckiebot.on_event(DuckieBotEvent.CAR_DETECTED)
        elif msg.data == 0:
            self._duckiebot.on_event(DuckieBotEvent.CAR_REMOVED)

    def FSM_state_callback(self, msg: FSMState):
        rospy.loginfo(f"FSM state changed to: {msg.state}")
        if msg.state == STOPPING_SUCCESS_FSM_STATE:
            self._duckiebot.on_event(DuckieBotEvent.BOT_BECOMES_STOPPED)
        elif msg.state == STOPPING_FAILURE_FSM_STATE:
            self._duckiebot.on_event(DuckieBotEvent.STOPPING_FAILURE)
        elif msg.state == OVERTAKING_SUCCESS_FSM_STATE:
            self._duckiebot.on_event(DuckieBotEvent.OVERTAKING_SUCCESS)
        elif msg.state == OVERTAKING_FAILURE_FSM_STATE:
            self._duckiebot.on_event(DuckieBotEvent.OVERTAKING_FAILURE)
        elif msg.state == TURNING_SUCCESS_FSM_STATE:
            self._duckiebot.on_event(DuckieBotEvent.TURNING_SUCCESS)
        elif msg.state == TURNING_FAILURE_FSM_STATE:
            self._duckiebot.on_event(DuckieBotEvent.TURNING_FAILURE)
        elif msg.state == APPROACHING_SIGN_SUCCESS_FSM_STATE:
            self._duckiebot.on_event(DuckieBotEvent.APPROACHING_SIGN_SUCCESS)
        elif msg.state == APPROACHING_SIGN_FAILURE_FSM_STATE:
            self._duckiebot.on_event(DuckieBotEvent.APPROACHING_SIGN_FAILURE)

    def april_tag_callback(self, msg: AprilTagDetectionArray):
        # Process the AprilTag detections
        for detection in msg.detections:
            id = detection.tag_id
            self._duckiebot._sign_tag_id = id  # Store the detected tag ID in the Duckiebot instance
            if self.is_stop_sign_id(id):
                self._duckiebot.on_event(DuckieBotEvent.STOP_SIGN_DETECTED)
            elif self.is_left_intersection_sign_id(id):
                self._duckiebot.on_event(DuckieBotEvent.TURN_LEFT_SIGN_DETECTED)
            elif self.is_right_intersection_sign_id(id):
                self._duckiebot.on_event(DuckieBotEvent.TURN_RIGHT_SIGN_DETECTED)
            elif self.is_t_intersection_sign_id(id):
                # Randomly choose between left and right turn for T-intersection
                import random
                self._duckiebot.on_event(DuckieBotEvent.TURN_LEFT_SIGN_DETECTED \
                    if random.choice([True, False]) else DuckieBotEvent.TURN_RIGHT_SIGN_DETECTED)
            else:
                rospy.loginfo(f"Unknown tag ID: {id}")

    def set_lane_following_parameters(self):
        rospy.set_param(LANE_CONTROLLER_NODE_V_BAR, V_BAR)
        rospy.set_param(LANE_CONTROLLER_NODE_K_D, K_D)
        rospy.set_param(LANE_CONTROLLER_NODE_K_THETA, K_THETA)
        rospy.set_param(LANE_CONTROLLER_NODE_K_ID, K_ID)
        rospy.set_param(LANE_CONTROLLER_NODE_K_IPHI, K_IPHI)
        rospy.set_param(LANE_CONTROLLER_NODE_THETA_THRES_MIN, THETA_THRES_MIN)
        rospy.set_param(LANE_CONTROLLER_NODE_THETA_THRES_MAX, THETA_THRES_MAX)
        rospy.loginfo("Lane following parameters set!")

    # Stop Robot before node has shut down. This ensures the robot keep moving with the latest velocity command
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Sends zero velocity to stop the robot
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def is_stop_sign_id(self, tag_id):
        return tag_id in STOP_SIGN_IDS
    def is_left_intersection_sign_id(self, tag_id):
        return tag_id in LEFT_INTERSECTION_SIGNS_IDS
    def is_right_intersection_sign_id(self, tag_id):
        return tag_id in RIGHT_INTERSECTION_SIGNS_IDS
    def is_t_intersection_sign_id(self, tag_id):
        return tag_id in T_INTERSECTION_SIGNS_IDS
    
    def run(self):
        rate = rospy.Rate(AUTOPILOT_UPDATE_FREQUENCY)
        while not rospy.is_shutdown():
            self._duckiebot.update()
            rate.sleep()

if __name__ == '__main__':
    try:
        autopilot = Autopilot()
        autopilot.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
