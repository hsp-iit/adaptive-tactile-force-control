import numpy
import rospy
from threading import Lock

# ROS Messages/services
from xela_2f_force_control.srv import SetFilterCutoff, SetFilterCutoffRequest, SetFilterCutoffResponse


class LPFilter():

    def __init__(self, cutoff, sampling_time):
        """Initialize class internal parameters and states."""

        self._sampling_time = sampling_time
        self._initialize_filter(cutoff)

        # Reset internal state
        self.reset()

        # Set up a service for changing the cutoff frequency
        self._control_serv = rospy.Service('~set_filter_cutoff', SetFilterCutoff, self.set_cutoff_callback)

        # Lock
        self._mutex = Lock()


    def _initialize_filter(self, cutoff):
        """Change the cutoff frequency."""

        constant = 2 * numpy.pi * self._sampling_time * cutoff
        self._alpha = constant / (constant + 1)


    def reset(self):
        """Reset internal state."""

        self._prev_state = None


    def filter(self, input_signal):
        """Return the output of the filter given the new value of the input signal."""

        output = None

        with self._mutex:
            if self._prev_state is None:
                output = input_signal
            else:
                # Implement y_i = alpha * u_i + (1 - alpha) * y_{i-1}
                output = self._alpha * input_signal + (1 - self._alpha) * self._prev_state

            self._prev_state = output

            return output


    def set_cutoff_callback(self, req):
        """Change the filter cutoff frequency."""

        if req.cutoff <= 0:
            return SetFilterCutoffResponse(False)

        with self._mutex:
            self.reset()
            self._initialize_filter(req.cutoff)
            return SetFilterCutoffResponse(True)
