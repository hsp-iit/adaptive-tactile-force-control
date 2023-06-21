import numpy
from ls_predictor import LSPredictor


class BatchLS():

    def __init__(self, window_size, order):
        """Constructor."""

        self._predictor = LSPredictor(order)
        self._order = order
        self._window_size = window_size

        self.reset()


    def add_point(self, x, k):
        """Add a point (x, k) to the window."""

        array = numpy.array(x).reshape(1, len(x))
        if self._data_x is None:
            self._data_x = array
        else:
            self._data_x = numpy.concatenate([self._data_x, array], axis = 0)
        self._data_k = numpy.append(self._data_k, k)

        if self._data_x.shape[0] > self._window_size:
            self._data_x = self._data_x[1:]
            self._data_k = self._data_k[1:]


    def coefficients(self):
        """Return the coefficients."""

        return self._coeff


    def estimate(self):
        """Estimate the coefficients."""

        if self._data_x is None:
            return False, None

        if self._data_x.shape[0] < self._order + 1:
            return False, None

        # Construct the regressor
        ones = numpy.ones((self._data_x.shape[0], 1))
        list_arrays = [ones]
        for i in range(1, self._order + 1):
            list_arrays.append(self._data_x ** i)

        X = numpy.concatenate(list_arrays, axis = 1)
        K = numpy.array([self._data_k]).T
        self._coeff = numpy.matmul(numpy.linalg.pinv(X), K)

        return True, self._coeff


    def evaluate(self, X):
        """Evaluate the local stiffness for a set of inputs."""

        return self._predictor.evaluate(X, self._coeff)


    def reset(self):
        """Reset the estimator."""

        self._coeff = None
        self._data_x = None
        self._data_k = numpy.array([])


    def is_estimate_valid(self):
        """Return the estimate status."""

        if self._coeff is None:
            return False

        return True
