"""
Unscented Kalman Filter
Author: Mark Shilton <mark.shilton@gmail.com>
Date: 2/12/2015

Notes: This is an unscented Kalman filter based on the algorithm from the book Probablistic Robotics
"""

class UnscentedKalmanFilter(object):
    """Initialise filter"""
    def __init__(self, state_dimensions, measurement_dimensions, rate_change, state_transistion_func, measurement_transistion_func):
        super(, self).__init__()
        # Number of variables in the state i.e x-position, y-poistion, x velociy, etc
        self.x_dim = state_dimensions
        # Number of variables in the measurement unction
        self.h_dim = measurement_dimensions
        
        self.dt = rate_change
        self.fx = state_transistion_func
        self.hx = measurement_transistion_func

        # Measurement noise function
        self.R = 0

        #Process noise function
        self.Q = 0

        self.lamda = 0

    """Function generates appropiate sigma points based on input variables"""
    def initSigmaPoints( alpha, beta, kappa):
        self.lamda = ((alpha**2)*(self.x_dim + kappa)) - self.x_dim
        return

    """Calculates the weights for the sigma points"""
    def sigmaPoints(mean, cov):

        return

    """Function that perform the prediction step"""
    def predict():

        return

    """Function that takes in a measurment value and use it to update the filter"""
    def update(measurement):

        return
