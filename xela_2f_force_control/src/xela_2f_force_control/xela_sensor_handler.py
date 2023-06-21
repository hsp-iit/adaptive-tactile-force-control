import copy
import numpy
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from threading import Lock
from xela_server.msg import xServerMsg


class XelaSensorHandler():

    def __init__(self):
        """Initialize the class properties and variables.
        Some parameters are parsed from the ROS parameter server."""

        # Load rosparam parameters from private names
        self._number_of_sensors = 2
        self._sensor_topic = rospy.get_param('~xela_sensor_topic', default = '/xela/sensors')

        # Setup service for bias reset
        self._bias_reset_service = rospy.Service('~xela_sensor_hander/reset_bias', Trigger, self._reset_bias_callback)

        # Bias data
        self._n_bias_samples = 50
        self._bias_samples = {}
        self._bias = {}

        # Sensor data
        self._sensor_data = {}

        # Mutex
        self._mutex = Lock()


    def wait_until_ready(self):
        """Wait until the sensor feedback is ready. Return False if waiting fails."""

        data = {}

        # Define temporary callback, subscriber and variable
        self._wait_msg = None
        def wait_cb(data):
            self._wait_msg = copy.deepcopy(data)
        wait_sub = rospy.Subscriber(self._sensor_topic, xServerMsg, wait_cb)

        # Wait for any message to show up for each sensor
        rospy.loginfo('Waiting for sensor feedback to be ready.')
        while len(data) != self._number_of_sensors:
            msg = self._wait_msg
            if msg:
                if msg.sensor not in data:
                    rospy.loginfo('Sensor feedback is active for sensor ' + str(msg.sensor) + '.')

                # Copy sensor data using sensor number as index
                data[msg.sensor] = copy.deepcopy(msg)

        # Cleanup
        wait_sub.unregister()
        del self._wait_msg

        rospy.loginfo('Sensor feedback is active for all sensors.')

        return True


    def setup(self):
        """Setup the sensor feedback."""

        # Wait for the sensor feedback to be ready
        if not self.wait_until_ready():
            raise RuntimeError('Error while waiting for the sensor feedback to be ready.')

        # Activate bias acquisition callback
        bias_sub = rospy.Subscriber(self._sensor_topic, xServerMsg, self._sensor_callback)

        # Stop until bias acquisition
        bias_aquisition_cycle = rospy.Duration(0.01)
        while not rospy.is_shutdown() and not self._bias_samples_acquired():
            rospy.sleep(bias_aquisition_cycle)


    def reset_bias(self):
        """Reset the bias."""

        # Reset bias data
        self._bias_samples = {}
        self._bias = {}

        # Stop until bias acquisition
        bias_aquisition_cycle = rospy.Duration(0.01)
        while not rospy.is_shutdown() and not self._bias_samples_acquired():
            rospy.sleep(bias_aquisition_cycle)


    def msg_to_matrix(self, data):
        """Convert a sensor message to a (sensor_number, numpy matrix) tuple."""

        sensor_number = data.sensor
        matrix = numpy.zeros(shape = (4, 6, 3), dtype = numpy.float32)

        # Specify a static assignment in order to respect the following layout
        #
        # Sample of the Left sensor
        #
        #           (         )
        #           ( Robotiq ) ( Realsense ) ---> camera optical axis
        #           (         )
        #              |  |
        #              |  |------> Left sensor   3  2  1  0   ------> Matrix
        #           ___|__|___                   __________
        #          |18 12 6 0 |                 |          | 0
        #          |19 13 7 1 |                 |          | 1
        #          |20 14 8 2 |                 |          | 2
        #          |21 13 9 3 |                 |          | 3
        #          |22 16 10 4|                 |          | 4
        #          |23 17 11 5|                 |          | 5
        #          |__________|                 |__________|

        for i in range(4):
            for j in range(6):
                data_point = data.points[6 * i + j]
                matrix[i][j][0] = data_point.point.x
                matrix[i][j][1] = data_point.point.y
                matrix[i][j][2] = data_point.point.z

        return sensor_number, matrix


    def get_sensor_data(self):
        """Return the last sensors data as received in the sensor callback."""

        data = {}

        with self._mutex:
            if len(self._sensor_data) == self._number_of_sensors:
                data = self._sensor_data

        return data


    def _bias_samples_acquired(self):
        """Assess whether the number of bias samples has been acquired."""

        with self._mutex:

            if self._bias:
                return True

            if len(self._bias_samples.keys()) < self._number_of_sensors:
                return False

            acquired = True
            for sensor_number in self._bias_samples.keys():
                acquired &= (self._bias_samples[sensor_number].shape[3] == self._n_bias_samples)

        return acquired


    def _set_sensor_bias(self):
        """Setup the bias for all sensors. Any processing should happen in this function."""

        with self._mutex:

            for sensor_number in self._bias_samples.keys():
                self._bias[sensor_number] = numpy.mean(self._bias_samples[sensor_number], axis = 3)
                rospy.loginfo('Bias for sensor ' + str(sensor_number) + ' is:')
                rospy.loginfo(self._bias[sensor_number])


    def _sensor_callback(self, data):
        """Define a different callback behaviour according to whether bias has been acquired or not"""

        if self._bias:
            self._sensor_feedback_callback(data)
        else:
            self._sensor_bias_callback(data)


    def _sensor_bias_callback(self, data):
        """Callback to be used while setting up sensor bias"""

        sensor_number, matrix = self.msg_to_matrix(data)

        # Bias is obtained by averaging multiple readings
        # Readings are saved as a (4, 6, 3, n_readings) shaped tensor
        # and then averaged
        with self._mutex:
            if not sensor_number in self._bias_samples.keys():
                # no bias samples for sensor yet
                self._bias_samples[sensor_number] = numpy.expand_dims(matrix, axis = 3)
            elif self._bias_samples[sensor_number].shape[3] < self._n_bias_samples:
                # stop at n_bias_samples
                self._bias_samples[sensor_number] = numpy.concatenate((self._bias_samples[sensor_number], numpy.expand_dims(matrix, axis = 3)), axis = 3)

        # If all bias samples have been acquired, set the sensor bias
        if self._bias_samples_acquired():
            self._set_sensor_bias()
            rospy.loginfo("Bias acquired, sensor feedback established.")


    def _sensor_feedback_callback(self, data):
        """Sensor callback. Data is acquired and unbiased."""

        sensor_number, matrix = self.msg_to_matrix(data)

        with self._mutex:
            self._sensor_data[sensor_number] = matrix - self._bias[sensor_number]


    def _reset_bias_callback(self, req):
        """Reset bias service callback."""

        self.reset_bias()

        return TriggerResponse(success = True, message="Bias reset.")
