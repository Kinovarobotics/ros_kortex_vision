from scipy.signal import lfilter
import matplotlib.pyplot as plt
import numpy as np

class ValueFilter:
    """
    Filters the values over time
    """
    def __init__(self, n:int, a:float, iterations:int, valueCount=2):
        self.buff = np.zeros((valueCount, iterations))
        self.n = n
        self.a = a
        self.b = [1.0 / n] * n 
        self.iterations = iterations
        self.index = 0

    def write(self, *args):
        for i, arg in enumerate(args):
            self.buff[i, self.index] = arg
        self.index = (self.index + 1) % self.iterations

    def fill(self, *args):
        for ii, _ in enumerate(self.buff[0,:]):
            self.buff[:,ii] = args

    @property
    def value(self):
        """
        Filtered value
        """
        res = list()
        for bf in self.buff:
            res.append(lfilter(self.b, self.a, bf)[-1])
        return tuple(res)