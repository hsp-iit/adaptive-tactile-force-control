#!/usr/bin/env python
import copy
import numpy
import rospy
import time
from datetime import datetime
from enum import Enum
from rospy.numpy_msg import numpy_msg
from threading import Lock
from joblib import load, dump

# Import components
from batch_ls import BatchLS
from controllers import AdaptiveProportionalController, EstimatorTesterController, ProportionalController
from lp_filter import LPFilter
from robotiq_gripper_handler import RobotiqGripperHandler
from stiffness_estimator import StiffnessEstimator
from xela_sensor_handler import XelaSensorHandler

# ROS Messages/services
from franka_msgs.msg import FrankaState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from xela_2f_force_control.msg import SetPointTarget, ControllerStatus, LoggerCommand
from xela_2f_force_control.srv import SetContactPosition, SetContactPositionRequest, SetContactPositionResponse
from xela_2f_force_control.srv import SetControllerGain, SetControllerGainRequest, SetControllerGainResponse
from xela_2f_force_control.srv import SetControllerType, SetControllerTypeRequest, SetControllerTypeResponse
from xela_2f_force_control.srv import SetControllerStatus, SetControllerStatusRequest, SetControllerStatusResponse
from xela_2f_force_control.srv import SetLoggerCommand, SetLoggerCommandRequest, SetLoggerCommandResponse
from xela_2f_force_control.srv import SetLoggerPath, SetLoggerPathRequest, SetLoggerPathResponse


class ControllerState(Enum):
    RUNNING = ControllerStatus.CONTROLLER_STATUS_RUNNING
    STOPPED = ControllerStatus.CONTROLLER_STATUS_STOPPED
    FAULTED = ControllerStatus.CONTROLLER_STATUS_FAULTED
    FIND_CONTACT = ControllerStatus.CONTROLLER_STATUS_FIND_CONTACT
    REFINE_CONTACT = ControllerStatus.CONTROLLER_STATUS_REFINE_CONTACT
    FOUND_CONTACT = ControllerStatus.CONTROLLER_STATUS_FOUND_CONTACT
    OPEN = ControllerStatus.CONTROLLER_STATUS_OPEN


class LoggerCommand(Enum):
    RUN = LoggerCommand.LOGGER_COMMAND_RUN
    STOP = LoggerCommand.LOGGER_COMMAND_STOP
    SAVE = LoggerCommand.LOGGER_COMMAND_SAVE


class Controller():

    def __init__(self):
        """Initialize class properties and internal parameters.
        Some parameters are parsed from the ROS parameter server.
        """

        rospy.init_node('xela_2f_force_controller_node')

        # Load rosparam parameters from private names
        self._controller_frequency = rospy.get_param('~control_frequency', default = 30)
        self._controller_strategy = rospy.get_param('~control_strategy', default = 'adaptive_proportional')
        self._controller_gain = rospy.get_param('~control_gain', default = 0.0001)

        self._contact_threshold = rospy.get_param('~contact_threshold', default = 5.0)
        self._contact_velocity = rospy.get_param('~contact_velocity', default = 0.0025)

        self._estimation_frequency = rospy.get_param('~estimation_frequency', default = 190)
        self._estimation_window = rospy.get_param('~estimation_window', default = 100)

        self._use_filter = rospy.get_param('~use_filter', default = False)
        self._filter_cutoff = rospy.get_param('~filter_cutoff', default = 5.0)

        self._logger_path = rospy.get_param('~logger_path', default = './')

        self._taxel_max_x = rospy.get_param('~taxel_max_x', default = 250.0)
        self._taxel_max_y = rospy.get_param('~taxel_max_y', default = 250.0)
        self._taxel_max_z = rospy.get_param('~taxel_max_z', default = 5000.0)
        self._taxel_activation = rospy.get_param('~taxel_activation', default = 0.0)

        self._franka_state_topic = rospy.get_param('~franka_state_topic', default = '/franka_state_controller/franka_states')
        self._reset_target = rospy.get_param('~reset_target', default = False)

        # Fixed parameters
        self._estimation_order = 2

        # Initialize controller state
        self._controller_state = ControllerState.STOPPED

        # Set up a service for logging
        self._logger_command = LoggerCommand.STOP
        self._logger_data = []
        self._logger_header = 'TIME ' + \
                              'TACTILE_FEEDBACK_RAW TACTILE_FEEDBACK TACTILE_SETPOINT ' + \
                              'GRIPPER_HOME GRIPPER_FEEDBACK GRIPPER_SETPOINT ' + \
                              'ADAPTIVE_COMMAND_STATUS ADAPTIVE_TACTILE_FEEDBACK ADAPTIVE_TACTILE_PREDICTION ADAPTIVE_GRIPPER_OFFSET ' + \
                              'ESTIMATION_X ESTIMATION_X_PREV ESTIMATION_Y ESTIMATION_Y_PREV ESTIMATION_ORDER ' + \
                              'COEFFS_0 COEFFS_1 COEFFS_2'
        self._logger_command_serv = rospy.Service('~set_logger_command', SetLoggerCommand, self.logger_command_callback)
        self._logger_path_serv = rospy.Service('~set_logger_path', SetLoggerPath, self.logger_path_callback)

        # Set up a service for changing the controller gain
        self._ctl_gain_serv = rospy.Service('~set_controller_gain', SetControllerGain, self.controller_gain_callback)

        # Set up a service for changing the controller type
        self._ctl_type_serv = rospy.Service('~set_controller_type', SetControllerType, self.controller_type_callback)

        # Set up a service for changing the contact position
        self._contact_position_serv = rospy.Service('~set_contact_position', SetContactPosition, self.contact_position_callback)

        # Fault button status
        self._fault_button_timer = 0.0

        # Lock
        self._mutex = Lock()

        # Gripper setpoint
        self._gripper_setpoint = None
        self._gripper_setpoint_expired = True

        # Setpoint
        self._setpoint = None

        # Storage for collecting the data to be logged
        self._collected_data = []
        self._sensors_data = {}
        self._current_time = None


    def is_fault_button_available(self):
        """Check if the fault button is alive."""

        with self._mutex:
            return self._fault_button_timer > 0.0


    def fault_button_update_time(self, delta_time):
        """Decrement the fault button timer."""

        with self._mutex:
            self._fault_button_timer -= delta_time

            # If negative, wrap it
            if self._fault_button_timer < 0.0:
                self._fault_button_timer = -1.0


    def is_control_time(self, delta_time):
        """Check if it is time to execute the control action."""

        if self._control_time > (1.0 / self._controller_frequency):
            self._control_time = 0.0
            return True
        else:
            self._control_time += delta_time
            return False


    def setup(self):
        """Setup topics, publishers, rate-limiter, gripper and sensors."""

        # Subscribe to required topics
        rospy.Subscriber(self._franka_state_topic, FrankaState, self.button_state_callback)
        rospy.Subscriber('/xela_2f_force_setpoint/setpoint', SetPointTarget, self.setpoint_callback)

        # Initialize publishers
        self._sensor_feedback_pub = rospy.Publisher('~sensor_feedback', Float32, queue_size = 10)
        self._commanded_gripper_pub = rospy.Publisher('~commanded_gripper_state', Float32, queue_size = 10)
        self._object_contact_pub = rospy.Publisher('~object_contact_position', Float32, queue_size = 10)

        # Set up a control service
        self._control_serv = rospy.Service('~set_controller_status', SetControllerStatus, self.controller_status_callback)

        # Setup the rate object according to the specified estimation frequency
        # and the variable holding the time elapsed from the last control action
        self._rate = rospy.Rate(self._estimation_frequency)
        self._control_time = 0.0

        # Initialize gripper
        self._gripper = RobotiqGripperHandler()
        self._gripper.setup()

        # Initialize stiffness estimator
        estimator = BatchLS(self._estimation_window, self._estimation_order)
        self._estimator = StiffnessEstimator(estimator, 30, 15)

        # Initialize controller
        self.initialize_controller(self._controller_strategy, self._controller_gain)

        # Initialize sensors
        self._sensor = XelaSensorHandler()
        self._sensor.setup()

        # Initialize low-pass filter
        self._filter = None
        if self._use_filter:
            self._filter = LPFilter(self._filter_cutoff, 1.0 / self._estimation_frequency)

        # Make sure the required data is available
        rospy.loginfo('Waiting for controller data to be available.')
        while not rospy.is_shutdown():
            if (self._gripper.get_position() is not None) and (len(self._sensor.get_sensor_data()) > 0):
                rospy.loginfo('Controller data is available.')
                break
            self._rate.sleep()

        # Give an initial value to the desired gripper position
        gripper_position = self._gripper.get_position()
        self._gripper_setpoint = gripper_position

        # Initialize the object contact position with a placeholder
        self._gripper_position_contact = -1


    def initialize_controller(self, control_type, control_gain):
        """Initialize the controller given the control gain."""

        if control_type == 'adaptive_proportional':
            rospy.loginfo('Initializing the adaptive proportional control strategy (k_p = ' + str(control_gain) + ').')
            self._controller = AdaptiveProportionalController(control_gain, 1.0 / self._controller_frequency, self._gripper.get_resolution(), self._estimator)
        elif control_type == 'proportional':
            rospy.loginfo('Initializing the proportional control strategy (k_p = ' + str(control_gain) + ').')
            self._controller = ProportionalController(control_gain, 1.0 / self._controller_frequency)
        elif control_type == 'estimator_tester':
            rospy.loginfo('Initializing the estimator tester control strategy.')
            self._controller = EstimatorTesterController(self._gripper.get_home_position(), 0.01, 7, 400.0, 3, self._estimator)
        else:
            rospy.logerr('A valid control strategy should be specified. Closing.')
            exit(-1)

    def loop(self):
        """The main controller loop."""

        # Initialize iteration time
        delta_time = 0.0

        while not rospy.is_shutdown():

            self._current_time = rospy.get_time()

            # Make sure the fault button is alive
            self.fault_button_update_time(delta_time)
            if not self.is_fault_button_available():
                self._controller_state = ControllerState.FAULTED

                rospy.logwarn('Not hearing from the fault button, is the franka_control node running?')

                rospy.sleep(0.1)
                delta_time = rospy.get_time() - self._current_time
                continue

            is_control_time = self.is_control_time(delta_time)

            stiffness_coeffs = None
            stiffness_meas = None
            adaptive_controller_data = None

            # Get data from the sensors
            sensors = self._sensor.get_sensor_data()
            if not self.is_sensor_safe(sensors):
                with self._mutex:
                    rospy.logwarn('Exceeding maximum taxels tolerances. Faulting the controller.')
                    self._controller_state = ControllerState.FAULTED
            mean_1, mean_2, mean_12 = self.evaluate_sensors_mean(sensors)

            # Evaluate the feedback signals
            feedback_raw = mean_12
            if self._filter is not None:
                feedback = self._filter.filter(feedback_raw)
            else:
                feedback = feedback_raw

            # Publish relevant data
            self._sensor_feedback_pub.publish(feedback)
            if is_control_time:
                self._object_contact_pub.publish(self._gripper_position_contact)

            # State machine
            with self._mutex:
                gripper_position = self._gripper.get_position()

                if self._controller_state == ControllerState.RUNNING:

                    # If the object contact position has not been detected yet, refuse to switch to RUNNING mode
                    if self._gripper_position_contact == -1:
                        self._controller_state = ControllerState.STOPPED
                        rospy.logwarn('The contact with the object has not been established yet. The controller will be stopped.')

                    elif self._setpoint is not None:

                        # Estimate stiffness
                        if gripper_position < self._gripper_position_contact:
                            x = abs(gripper_position - self._gripper_position_contact)
                            y = feedback_raw
                            stiffness_meas = self._estimator.set_measurement(x, y)
                        valid_coeffs, stiffness_coeffs = self._estimator.estimate()

                        if is_control_time:
                            # Evaluate control velocity
                            self._controller.step(feedback, self._setpoint, gripper_position, self._gripper_position_contact)
                            if self._controller_strategy == 'adaptive_proportional' or \
                               self._controller_strategy == 'estimator_tester':
                                adaptive_controller_data = self._controller.get_status_data()

                            # The estimator tester provides information when the experiment is completed
                            if self._controller_strategy == 'estimator_tester':
                                if self._controller.is_test_complete():
                                    rospy.loginfo('Estimation test complete')
                                    self.save_data()
                                    self._logger_command = LoggerCommand.STOP
                                    self._controller_state = ControllerState.STOPPED

                            # Integrate control velocity
                            self._gripper_setpoint += self._controller.get_command()

                            # Clamp gripper width
                            if self._gripper_setpoint > 0.085:
                                self._gripper_setpoint = 0.085
                            elif self._gripper_setpoint < 0:
                                self._gripper_setpoint = 0.0

                            # Send gripper command
                            self._gripper.go_position(self._gripper_setpoint)
                            self._commanded_gripper_pub.publish(self._gripper_setpoint)

                elif self._controller_state == ControllerState.FIND_CONTACT:

                    if is_control_time:

                        self._gripper_setpoint -= self._contact_velocity * (1.0 / self._controller_frequency)

                        if (mean_1 > self._contact_threshold) and (mean_2 > self._contact_threshold):
                            final_target = gripper_position
                            self._gripper.go_position(final_target)
                            self._controller_state = ControllerState.REFINE_CONTACT

                            rospy.loginfo('Detected contact position at ' + str(final_target))
                        else:
                            self._gripper.go_position(self._gripper_setpoint)
                            self._commanded_gripper_pub.publish(self._gripper_setpoint)

                elif self._controller_state == ControllerState.REFINE_CONTACT:
                    if feedback_raw > 10:
                        final_target = gripper_position + self._gripper.get_resolution()
                        self._gripper.go_position(final_target)
                        rospy.sleep(0.1)
                    else:
                        self._gripper_position_contact = gripper_position
                        rospy.loginfo('Refined contact position at ' + str(self._gripper_position_contact))
                        self._controller_state = ControllerState.FOUND_CONTACT

                elif self._controller_state == ControllerState.FOUND_CONTACT:
                    # Idle, refresh gripper position but leave object contact position unaltered
                    self._gripper_setpoint = gripper_position
                    self._setpoint = None

                elif self._controller_state == ControllerState.FAULTED:
                    # Open gripper
                    self._gripper.home(block = True)

                    # Reset variables
                    self._gripper_setpoint = gripper_position
                    self._gripper_position_contact = -1
                    self._setpoint = None

                elif self._controller_state == ControllerState.STOPPED:
                    # Reset variables
                    self._gripper_setpoint = gripper_position
                    self._gripper_position_contact = -1
                    self._setpoint = None

                    # Reset estimator
                    self._estimator.reset()
                    self._controller.reset()

                elif self._controller_state == ControllerState.OPEN:
                    # Open gripper
                    self._gripper.home(block = True)

                    # Move to STOPPED
                    self._controller_state = ControllerState.STOPPED

                # Handle data collection
                if self._logger_command == LoggerCommand.RUN:
                    self.collect_data(sensors, feedback_raw, feedback, self._setpoint, \
                                      gripper_position, self._gripper_setpoint, \
                                      stiffness_meas, \
                                      stiffness_coeffs, \
                                      adaptive_controller_data)

                elif self._logger_command == LoggerCommand.STOP:
                    # Reset data storage
                    self.clean_data()

            self._rate.sleep()

            # Update the iteration time
            delta_time = rospy.get_time() - self._current_time


    def collect_data(self, \
                     tactile_sensors, tactile_feedback_raw, tactile_feedback, tactile_setpoint, \
                     gripper_feedback, gripper_setpoint, \
                     stiffness_meas,
                     stiffness_coeffs,
                     adaptive_controller_data
                     ):
        """Save the controller data streams."""

        tactile_setpoint_data = copy.copy(tactile_setpoint)
        gripper_setpoint_data = copy.copy(gripper_setpoint)

        # Set defaults

        if not tactile_setpoint_data:
            tactile_setpoint_data = -1

        if not gripper_setpoint_data:
            gripper_setpoint_data = -1

        adaptive_status = -1
        adaptive_feedback = -1
        adaptive_prediction = -1
        adaptive_gripper_offset = -1
        stiffness_meas_x = -1
        stiffness_meas_x_prev = -1
        stiffness_meas_y = -1
        stiffness_meas_y_prev = -1

        if adaptive_controller_data is not None:
            if adaptive_controller_data['status'] is not None:
                adaptive_status = adaptive_controller_data['status']
            if adaptive_controller_data['feedback'] is not None:
                adaptive_feedback = adaptive_controller_data['feedback']
            if adaptive_controller_data['prediction'] is not None:
                adaptive_prediction = adaptive_controller_data['prediction']
            if adaptive_controller_data['gripper_offset'] is not None:
                adaptive_gripper_offset = adaptive_controller_data['gripper_offset']

        if stiffness_meas is not None:
            stiffness_meas_x = stiffness_meas['x']
            stiffness_meas_x_prev = stiffness_meas['x_previous']
            stiffness_meas_y = stiffness_meas['y']
            stiffness_meas_y_prev = stiffness_meas['y_previous']

        # Append data
        new_data = [self._current_time, \
                    tactile_feedback_raw, tactile_feedback, tactile_setpoint_data, \
                    self._gripper_position_contact, gripper_feedback, gripper_setpoint_data, \
                    adaptive_status, adaptive_feedback, adaptive_prediction, adaptive_gripper_offset, \
                    stiffness_meas_x, stiffness_meas_x_prev, stiffness_meas_y, stiffness_meas_y_prev,
                    self._estimation_order]

        if stiffness_coeffs is not None:
            for i in range(stiffness_coeffs.shape[0]):
                new_data.append(stiffness_coeffs[i])
        else:
            for i in range(3):
                new_data.append(-1)

        new_data = numpy.array(new_data)

        self._collected_data.append(new_data)

        # Append sensors data
        first_insertion = (len(self._sensors_data) == 0)
        for i in tactile_sensors:
            if first_insertion:
                self._sensors_data[i] = numpy.expand_dims(tactile_sensors[i], axis = 3)
            else:
                self._sensors_data[i] = numpy.concatenate((self._sensors_data[i], numpy.expand_dims(tactile_sensors[i], axis = 3)), axis = 3)


    def clean_data(self):
        """Clean the collected data."""

        self._collected_data = []
        self._sensors_data = {}


    def save_data(self):
        """Save the collected data."""

        data = self._collected_data

        if len(data) == 0:
            # Nothing to save
            return

        output_data = numpy.array(data)
        root_path = self._logger_path + '/log_' + datetime.now().strftime('%d_%m_%Y_%H_%M_%S')
        numpy.savetxt(root_path + '.txt', output_data, header = self._logger_header)

        # Export the sensors data
        dump(self._sensors_data, root_path + '_sensors.joblib')


    def evaluate_sensors_mean(self, sensors):
        """Evaluate a scalar mean over all the z channels."""

        means = []

        # Mean over z channel of sensing units for each sensor
        z_channel_index = 2
        for sensor_number in sensors:
            means.append(numpy.mean(sensors[sensor_number][:, :, z_channel_index]))

        # Return mean of the means
        return means[0], means[1], numpy.mean(means)


    def is_sensor_safe(self, sensors):
        """Check if the sensors values are within the tolerances specified by the users."""

        max_x = numpy.max([abs(sensors[1][:, :, 0]).max(), abs(sensors[2][:, :, 0]).max()])
        max_y = numpy.max([abs(sensors[1][:, :, 1]).max(), abs(sensors[2][:, :, 1]).max()])
        max_z = numpy.max([abs(sensors[1][:, :, 2]).max(), abs(sensors[2][:, :, 2]).max()])

        is_safe = True
        if max_x >= self._taxel_max_x:
            rospy.logwarn('Exceeding maximum taxels tolerances along x-axis (>= ' + str(self._taxel_max_x) + '.')
            is_safe = False

        if max_y >= self._taxel_max_y:
            rospy.logwarn('Exceeding maximum taxels tolerances along y-axis (>= ' + str(self._taxel_max_y) + '.')
            is_safe = False

        if max_z >= self._taxel_max_z:
            rospy.logwarn('Exceeding maximum taxels tolerances along z-axis (>= ' + str(self._taxel_max_z) + '.')
            is_safe = False

        return is_safe


    def setpoint_callback(self, data):
        """Setpoint callback."""

        with self._mutex:
            self._setpoint = data.setpoint_target

            # This is the beginning of the trajectory
            # The gripper desired trajectory needs to be reset to the current configuration
            # This mechanism make sure the gripper desired position is reset
            # even if the very first frame (i.e. seq == 0) is lost
            if data.setpoint_header.seq < 10:
                if self._gripper_setpoint_expired:
                    if self._reset_target:
                        self._gripper_setpoint = self._gripper.get_position()
                    self._gripper_setpoint_expired = False
            # Reset the status for the next trajectory
            else:
                self._gripper_setpoint_expired = True


    def button_state_callback(self, data):
        """Franka fault button callback. The controller must be restarted via setpoint or service.

        Info: data.robot_mode can be:
        ROBOT_MODE_OTHER                          /
        ROBOT_MODE_IDLE                           Robot is being controlled via ROS (blue LED)
        ROBOT_MODE_MOVE                           Not tested
        ROBOT_MODE_GUIDING                        Robot is being hand-guided by the user (white LED)
        ROBOT_MODE_REFLEX                         Robot is automatically stopped because of a reflex (blue LED)
        ROBOT_MODE_USER_STOPPED                   Robot is in user-guided mode but not currently being hand-guided (white LED)
                                                  Robot registers a conflict in soft-stops status (e.g. fault released and hand-guidance button being pressed) (purple LED)
        ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY=6     Not tested
        """

        with self._mutex:

            # Reset the fault button timer
            self._fault_button_timer = 1.0

            # Handle the status of the fault button
            if ((data.robot_mode == FrankaState.ROBOT_MODE_REFLEX) or (data.robot_mode == FrankaState.ROBOT_MODE_GUIDING) or (data.robot_mode == FrankaState.ROBOT_MODE_USER_STOPPED)) and self._controller_state != ControllerState.FAULTED:
                self._controller_state = ControllerState.FAULTED
                rospy.loginfo('Controller faulted!')


    def controller_status_callback(self, req):
        """Request a controller status change. Return false if status is invalid."""

        req_status = req.target_status.controller_status

        # req_status is a int8, we want the corresponding ControllerState entry
        # If invalid, return False
        with self._mutex:
            for st in ControllerState:
                if st.value == req_status:
                    self._controller_state = st
                    return SetControllerStatusResponse(True)
            return SetControllerStatusResponse(False)


    def logger_command_callback(self, req):
        """Command the logger."""

        req_command = req.logger_command.logger_command

        with self._mutex:
            for command in LoggerCommand:
                if command.value == req_command:

                    if command == LoggerCommand.SAVE:
                        # Save data
                        self.save_data()
                    else:
                        self._logger_command = command

                    return SetLoggerCommandResponse(True)
            return SetLoggerCommandResponse(False)


    def logger_path_callback(self, req):
        """Set the logger path."""

        path = req.path

        with self._mutex:
            self._logger_path = path

        return SetLoggerPathResponse(True)


    def controller_gain_callback(self, req):
        """Set the controller gain."""

        gain = req.gain

        with self._mutex:
            if self._controller_state != ControllerState.STOPPED:
                rospy.logerr('The gain can be changed only if the controller is stopped.')
                return SetControllerGainResponse(False)
            else:
                self._controller_gain = gain
                self.initialize_controller(self._controller_strategy, self._controller_gain)
                return SetControllerGainResponse(True)


    def controller_type_callback(self, req):
        """Set the controller gain."""

        type = req.type

        with self._mutex:
            if self._controller_state != ControllerState.STOPPED:
                rospy.logerr('The controller type can be changed only if the controller is stopped.')
                return SetControllerTypeResponse(False)
            else:
                self._controller_strategy = type
                self.initialize_controller(self._controller_strategy, self._controller_gain)
                return SetControllerTypeResponse(True)


    def contact_position_callback(self, req):
        """Set the controller contact position."""

        position = req.position

        with self._mutex:
            if self._controller_state != ControllerState.STOPPED:
                rospy.logerr('The contact position can be changed only if the controller is stopped.')
                return SetContactPositionResponse(False)
            else:
                rospy.loginfo('The contact position has been set from service to ' + str(position) + '.')

                self._gripper_position_contact = position
                self._gripper.go_position(self._gripper_position_contact)
                self._controller_state = ControllerState.FOUND_CONTACT

                return SetContactPositionResponse(True)


def main():
    controller = Controller()
    controller.setup()
    controller.loop()
