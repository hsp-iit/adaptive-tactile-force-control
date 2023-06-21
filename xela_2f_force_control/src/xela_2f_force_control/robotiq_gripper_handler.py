import actionlib
import copy
import rospy
from threading import Lock
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal, RobotiqGripperStatus


class RobotiqGripperHandler():

    def __init__(self):
        """Initialize the class properties and variables.
        Some parameters are parsed from the ROS parameter server."""

        # Load rosparam parameters from private names
        self._mode = rospy.get_param('~gripper_mode', default = 'actionlib')
        self._force = rospy.get_param('~force', default = 10.0)
        self._speed = rospy.get_param('~speed', default = 0.01)
        self._force_home = rospy.get_param('~force_home', default = 10.0)
        self._speed_home = rospy.get_param('~speed_home', default = 0.03)
        self._pos_home = rospy.get_param('~pos_home', default = 0.085)
        self._status_topic = rospy.get_param('~gripper_status_topic', default = '/robotiq_2f_action_server/gripper_status')

        # Gripper status
        self._status = None

        # Mutex
        self._mutex = Lock()


    def wait_until_ready(self):
        """Wait until the gripper is ready to receive commands and send feedback. Return False if waiting fails."""

        outcome = True

        # Wait until the gripper is ready to receive commands
        if self._mode == 'actionlib':
            rospy.loginfo('Waiting for gripper action server to be ready.')
            outcome &= self._action.wait_for_server(timeout = rospy.Duration(10.0))
            if outcome:
                rospy.loginfo('Gripper action server is ready.')
            else:
                rospy.logerr('Error while waiting for gripper action server.')

        # Wait until the gripper is ready to send its feedback
        try:
            rospy.loginfo('Waiting for gripper feedback to be ready.')
            rospy.wait_for_message(self._status_topic, RobotiqGripperStatus, timeout = rospy.Duration(10.0))
        except rospy.ROSException:
            outcome = False

        if outcome:
            rospy.loginfo('Gripper feedback is active.')
        else:
            rospy.logerr('Error while waiting for gripper feedback to be ready.')

        return outcome


    def setup(self):
        """Setup the gripper."""

        # Manage gripper actuation
        valid_modes = ['actionlib', 'topic']
        if self._mode not in valid_modes:
            raise NotImplementedError('Mode ' + mode + ' not available for gripper handling.')

        self._action = None

        if self._mode == 'actionlib':
            self._action = actionlib.SimpleActionClient('command_robotiq_action', CommandRobotiqGripperAction)
        elif self.mode  == 'topic':
            raise NotImplementedError

        # Wait for the gripper to be ready
        if not self.wait_until_ready():
            raise RuntimeError('Error while waiting for the gripper to be ready.')

        # Manage gripper status feedback
        rospy.Subscriber(self._status_topic, RobotiqGripperStatus, self.status_callback)

        # Open the gripper
        self.home()


    def command(self, pos, speed, force, block = None):
        """Command a desired position for the gripper."""

        if self._mode == 'actionlib':
            Robotiq.goto(self._action, pos = pos, speed = speed, force = force, block = block)
        elif self._mode == 'topic':
            raise NotImplementedError


    def home(self, block = True):
        """Command the gripper to home position."""

        return self.command(self._pos_home, self._speed_home, self._force_home, block)


    def get_home_position(self):
        """Return the gripper home position."""

        return self._pos_home


    def go_position(self, pos):
        """Command the gripper to the desired position."""

        return self.command(pos, self._speed, self._force)


    def status_callback(self, data):
        """Gripper state callback."""

        with self._mutex:
            self._status = copy.deepcopy(data)


    def get_position(self):
        """Return the last gripper position as received in the status callback."""

        position = None

        with self._mutex:
            if self._status is not None:
                position = self._status.position

        return position


    def get_resolution(self):
        """Return the smallest distance the gripper fingers can travel."""

        return 0.00037445127964019775
