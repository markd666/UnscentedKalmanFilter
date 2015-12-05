
from Unscented_kalman_filter import UnscentedKalmanFilter

ukf = UnscentedKalmanFilter(state_dimensions=2, measurement_dimensions=1, rate_change=1, state_transistion_func="null", measurement_transistion_func="null")

ukf.initSigmaPoints(alpha=2,beta=1,kappa=1)
