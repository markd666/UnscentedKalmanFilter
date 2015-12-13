"""
Unscented Kalman Filter
Author: Mark Shilton <mark.shilton@gmail.com>
Date: 2/12/2015

Notes: This is an unscented Kalman filter based on the algorithm from the book Probablistic Robotics
"""
import numpy as np

class UnscentedKalmanFilter(object):
    """Initialise filter"""
    def __init__(self, state_dimensions, measurement_dimensions, rate_change, state_transistion_func, measurement_transistion_func):

        # Number of variables in the state i.e x-position, y-poistion, x velociy, etc
        self.x_dim = np.zeros(state_dimensions)
        self.x_size = state_dimensions
        # Number of variables in the measurement unction
        self.h_dim = measurement_dimensions
        # How often the filter is called..
        self.dt = rate_change
        # Function that will convert
        self.fx = state_transistion_func
        self.hx = measurement_transistion_func

        # Measurement noise function
        self.R = np.eye(state_dimensions, dtype=float)

        #Process noise function
        self.Q = 0

        self.lamda = 0

        #Output
        print '{0}= {1}'.format('State Dimensions', np.size(self.x_dim))
        print '{0}= {1}'.format('Measurement Dimensions', np.size(self.h_dim))
        print '{0}= {1}'.format('Rate Change', self.dt)

    """Function generates appropiate sigma points based on input variables"""
    def initSigmaPoints(self, alpha, beta, kappa):
        #calculate the lamda value that will be used later on
        self.lamda = ((alpha**2)*(self.x_size + kappa)) - self.x_size

        # Work out the number of sigma points based on the number of state Dimensions
        # i.e 1 dimension equals 3 sigma points. 2 dimensions equals 5 sigm points
        self.sigmas = np.zeros((2*self.x_size +1,self.x_size))

        #print 'Number of Sigmas points is:'
        #print (self.sigmas)
        return

    """Calculates the weights for the sigma points"""
    def sigmaPoints(self, mean, cov):

        #Check that the means is the same dimensionality as the initialised UKF
        assert self.x_size == np.size(mean),  "expected size {}, but size is {}".format(
            self.x_size, np.size(mean) )

        # Make sure mean is a in the array format and convert if necessary
        if np.isscalar(mean):
            mean = np.asarray([mean])

        # Make sure he covariance is a matrix
        if np.isscalar(cov):
            cov = np.eye(n)*cov

        # Calculating the sigma points
        # Already found lamda from 'initSigmaPoints' function
        diff = np.sqrt((self.x_size + self.lamda)*cov)

        #sigma matrix already initialised in 'initSigmaPoints' function
        # The first sigma point is the mean
        self.sigmas[0] = mean

        for k in range(self.x_size):
            self.sigmas[k+1] = np.add(mean, diff[k])
            self.sigmas[self.x_size + k + 1] = np.subtract(mean, diff[k])

        print ("Generated Sigma points")
        print self.sigmas

        # loop through

        return

    """Function that perform the prediction step"""
    def predict():

        return

    """Function that takes in a measurment value and use it to update the filter"""
    def update(measurement):

        return
