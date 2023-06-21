import numpy


class StiffnessEstimator:

    def __init__(self, estimator, time_window_size, wait_steps):
        """Initialize the estimator."""

        self._time_window_size = time_window_size
        self._wait_steps = wait_steps

        self._x_previous = None
        self._x_store = None
        self._x_previous_store = None
        self._y_window = numpy.array([])

        self._waiting = False
        self._wait_steps_remaining = 0

        self._estimator = estimator


    def set_measurement(self, x, y):
        """Set a new measurement (x, y).

           The points (x_previous, y_previous) and (x, y) are added to the set of data of interest.

           Return a dictionary made of {'x' : <value>, 'x_previous' : <value>, 'y' : <value>, 'y_previous' : <value>} if the measurement is accepted.
           Return None, othrwise."""

        output = None

        if self._x_previous == None:
            self._x_previous = x
            self._y_window = numpy.append(self._y_window, y)

            return

        if self._waiting:
            if self._wait_steps_remaining == 0:
                # Take the leftmost value as the previous value
                y_previous = self._y_window[0]

                self.add_point([self._x_previous_store], y_previous)
                self.add_point([self._x_store], y)

                self._waiting = False

                output = {'x' : self._x_store, \
                          'x_previous' : self._x_previous_store, \
                          'y' : y, \
                          'y_previous' : y_previous}
            else:
                self._wait_steps_remaining -= 1

        elif abs(x - self._x_previous) > 0:
            self._x_store = x
            self._x_previous_store = self._x_previous
            self._waiting = True
            self._wait_steps_remaining = self._wait_steps

        self._x_previous = x
        self._y_window = numpy.append(self._y_window, y)
        if self._y_window.shape[0] > self._time_window_size:
            self._y_window = self._y_window[1:]

        return output


    def add_point(self, x, y):
        """Add a point (x, y)."""

        self._estimator.add_point(x, y)


    def estimator(self):
        """Return the underlying estimator."""

        return self._estimator


    def estimate(self):
        """Estimate the coefficients"""

        return self._estimator.estimate()


    def evaluate(self, X):
        """Evaluate the stiffness"""

        return self._estimator.evaluate(X)


    def reset(self):
        """Reset the estimator"""

        self._estimator.reset()


    def is_estimate_valid(self):
        """Return the estimate status."""

        return self._estimator.is_estimate_valid()
