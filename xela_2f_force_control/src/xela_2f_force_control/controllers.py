import copy
import rospy
import numpy
from simple_pid import PID


class EstimatorTesterController:

    def __init__(self, starting_pose, minimum_value, wait_steps, max_force, number_cycles, predictor):
        """Initialize the open loop controller."""

        # Controller logic
        self.wait_steps = wait_steps
        self.delta = 0.001 * 0.4
        self.number_cycles = number_cycles

        # Controller state and limits
        self.starting_pose = starting_pose
        self.pose_upper = self.starting_pose
        self.pose_lower = minimum_value
        self.max_force = max_force

        # Predictor
        self.predictor = predictor

        self.reset()


    def do_step(self):
        """Check if it is time to step."""

        if self.wait_counter == 0:
            self.wait_counter = self.wait_steps
            return True
        else:
            self.wait_counter -= 1
            return False


    def reset(self):
        """Reset the controller."""

        # Controller logic
        self.wait_counter = self.wait_steps
        self.direction = -1
        self.command = 0.0
        self.cycles_counter = 0
        self.is_test_complete_value = False

        # Controller state and limits
        self.state = self.starting_pose

        # Storage
        self.last_prediction = None
        self.last_feedback = None
        self.last_gripper_offset = None
        self.last_status = None



    def step(self, feedback, desired, gripper_position, gripper_position_zero):
        """Perform a step of the controller."""

        # Set status
        self.last_status = 0

        if not self.do_step():
            self.command = 0.0
            return

        # Evaluate command
        self.command = self.delta * self.direction
        state = self.state + self.command

        # Handle motion/force boundaries
        pose_lower_reached = state < self.pose_lower
        pose_upper_reached = state > self.pose_upper
        force_reached = ((self.direction == -1) and (feedback > self.max_force))
        if pose_lower_reached or force_reached or pose_upper_reached:
            self.direction *= -1
            self.command = 0.0

            if force_reached:
                self.cycles_counter += 1

            if pose_upper_reached:
                if self.cycles_counter >= self.number_cycles:
                    self.is_test_complete_value = True

        self.state += self.command

        # Check conditions
        if self.command == 0:
            return

        if gripper_position >= gripper_position_zero:
            return

        if not self.predictor.is_estimate_valid():
            return

        # Evaluate equivalent slope
        coeffs = self.predictor.estimator().coefficients()
        offset = abs(gripper_position - gripper_position_zero)
        equivalent_slope = coeffs[1] + 2 * coeffs[2] * offset
        if equivalent_slope < 0:
            return

        # Evalute prediction
        predicted_f = feedback - equivalent_slope * self.command

        # Use physics based reasoning to exclude necessarily wrong estimates
        if ((self.command > 0) and (predicted_f > feedback)) or ((self.command < 0) and (predicted_f < feedback)):
            return

        self.last_status = 1
        self.last_prediction = predicted_f
        self.last_feedback = feedback
        self.last_gripper_offset = offset


    def get_command(self):
        """Get the next command (to be intended as a variation with respect to the current gripper position)."""

        return self.command


    def get_status_data(self):
        """Return relevant data describing the status of the controller."""

        data = {}
        data['status'] = self.last_status
        data['feedback'] = self.last_feedback
        data['prediction'] = self.last_prediction
        data['gripper_offset'] = self.last_gripper_offset

        return data


    def is_test_complete(self):

        return self.is_test_complete_value


class ProportionalController:

    def __init__(self, proportional_gain, sample_time):
        """Initialize the simple controller."""

        self.proportional_gain = proportional_gain
        self.sample_time = sample_time

        self.reset()


    def reset(self):
        """Reset the controller."""

        self.pid = PID(Kp = self.proportional_gain,
                        Kd = 0.0,
                        Ki = 0.0,
                        sample_time = self.sample_time,
                        setpoint = -10.0)


    def step(self, feedback, desired, *args):
        """Perform a step of the controller."""

        self.pid.setpoint = desired
        command = self.pid(feedback) * self.sample_time

        # Invert commanded error due to gripper logic
        self.command = -command


    def get_command(self):
        """Get the next command (to be intended as a variation with respect to the current gripper position)."""

        return self.command


class AdaptiveProportionalController:

    def __init__(self, proportional_gain, sample_time, delta_position, predictor):
        """Initialize the adaptive proportional controller."""

        self.proportional_gain = proportional_gain
        self.sample_time = sample_time
        self.delta_position = delta_position
        self.predictor = predictor

        self.reset()


    def reset(self):
        """Reset the controller."""

        self.pid = PID(Kp = self.proportional_gain,
                       Kd = 0.0,
                       Ki = 0.0,
                       sample_time = self.sample_time,
                       setpoint = -10.0)

        self.last_prediction = None
        self.last_feedback = None
        self.last_gripper_offset = None
        self.last_status = None
        self.command = 0


    def step(self, feedback, desired, gripper_position, gripper_position_zero):
        """Perform a time step of the controller."""

        # Set status
        self.last_status = 0

        # Evaluate command (- sign due to gripper logic)
        self.pid.setpoint = desired
        self.command = -self.pid(feedback) * self.sample_time

        # Check conditions
        if self.command == 0:
            return

        if gripper_position >= gripper_position_zero:
            return

        if not self.predictor.is_estimate_valid():
            return

        # Evaluate equivalent slope
        coeffs = self.predictor.estimator().coefficients()
        offset = abs(gripper_position - gripper_position_zero)
        equivalent_slope = coeffs[1] + 2 * coeffs[2] * offset
        if equivalent_slope < 0:
            return

        # Evalute prediction
        error = -(desired - feedback) # - sign due to gripper logic
        number_ticks = int(numpy.ceil(abs(error) / (self.delta_position * equivalent_slope))[0])
        expected_command =  numpy.sign(error) * number_ticks * self.delta_position
        predicted_f = feedback - equivalent_slope * expected_command

        # Use physics based reasoning to exclude necessarily wrong estimates
        if ((self.command > 0) and (predicted_f > feedback)) or ((self.command < 0) and (predicted_f < feedback)):
            return

        predicted_error = desired - predicted_f
        if abs(predicted_error) > abs(error):
            self.command = 0.0

        self.last_status = 1
        self.last_prediction = predicted_f
        self.last_feedback = feedback
        self.last_gripper_offset = offset


    def get_command(self):
        """Get the next command (to be intended as a variation with respect to the current gripper position)."""

        return self.command


    def get_status_data(self):
        """Return relevant data describing the status of the controller."""

        data = {}
        data['status'] = self.last_status
        data['feedback'] = self.last_feedback
        data['prediction'] = self.last_prediction
        data['gripper_offset'] = self.last_gripper_offset

        return data
