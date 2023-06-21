from ls_predictor import LSPredictor


class LSMemory():

    def __init__(self, coefficients, order):
        """Constructor."""

        self._coefficients = coefficients
        self._predictor = LSPredictor(order)


    def add_point(self, *args):
        """Placeholder."""
        pass


    def coefficients(self):
        """Return the coefficients."""

        return self._coefficients


    def estimate(self):
        """Placeholder."""

        return True, self._coefficients


    def evaluate(self, X):
        """Placeholder."""

        return self._predictor.evaluate(X, self._coefficients)
