import numpy


class LSPredictor():

    def __init__(self, order):
        """Constructor."""

        self._order = order


    def evaluate(self, inputs, coeffs):
        """Evaluate the local stiffness for a set of inputs."""

        ones = numpy.ones((inputs.shape[0], 1))

        lists_array = [ones]
        for i in range(1, self._order + 1):
            lists_array.append(inputs ** i)
        x = numpy.concatenate(lists_array, axis = 1)

        return numpy.matmul(x, coeffs)
