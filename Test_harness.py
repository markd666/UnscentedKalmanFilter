
from Unscented_kalman_filter import UnscentedKalmanFilter
import numpy as np

#Initialisa the unscented kalman filter
ukf = UnscentedKalmanFilter(state_dimensions=2, measurement_dimensions=1, rate_change=1, state_transistion_func="null", measurement_transistion_func="null")

#Initialise the filter points sigma point variables (alpha/beta/kappa)
ukf.initSigmaPoints(alpha=2,beta=1,kappa=1)

#set up mean and covariance variables
covariance = np.eye(2)
covariance *= 1.2
mean = np.array([1, 1])

#Test out generating sigma points based on the mean and covariance variables
ukf.sigmaPoints(mean, covariance)


#Initialisa the unscented kalman filter
ukf = UnscentedKalmanFilter(state_dimensions=1, measurement_dimensions=1, rate_change=1, state_transistion_func="null", measurement_transistion_func="null")

#Initialise the filter points sigma point variables (alpha/beta/kappa)
ukf.initSigmaPoints(alpha=2,beta=1,kappa=1)

#set up mean and covariance variables
covariance = np.eye(1)
covariance *= 1.2
mean = np.array([2])

#Test out generating sigma points based on the mean and covariance variables
ukf.sigmaPoints(mean, covariance)
