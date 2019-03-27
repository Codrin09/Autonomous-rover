from pykalman import UnscentedKalmanFilter, KalmanFilter
import numpy as np
import matplotlib.pyplot as plt
import time


def f(state, noise):
    return state #+ np.sin(noise)

def g(state, noise):
    return state #+ np.cos(noise)

# ukf = UnscentedKalmanFilter(transition_functions = [translateX, translateY], [g, R=0.1)    
# ukf = UnscentedKalmanFilter(lambda x, w: x + np.sin(w), lambda x, v: x + v, observation_covariance=0.1)
# (filtered_state_means, filtered_state_covariances) = ukf.filter([0, 1, 2])
# print(filtered_state_means, filtered_state_covariances)
def kalman():
    global time, time_before, n_real_time
    kf3 = KalmanFilter(transition_matrices = transition_matrix,
                    observation_matrices = observation_matrix,
                    initial_state_mean = initial_state_mean,
                    # em_vars=['transition_covariance', 'initial_state_covariance', 'observation_covariance']
                    )

    kf3 = kf3.em(measurements[:-n_real_time, :], n_iter=5)
    (filtered_state_means, filtered_state_covariances) = kf3.filter(measurements[:-n_real_time,:])

    print("Time to build and train kf3: %s seconds" % (time.time() - time_before))

    x_now = filtered_state_means[-1, :]
    P_now = filtered_state_covariances[-1, :]
    x_new = np.zeros((n_real_time, filtered_state_means.shape[1]))
    i = 0

    for measurement in measurements[-n_real_time:, :]:
        time_before = time.time()
        (x_now, P_now) = kf3.filter_update(filtered_state_mean = x_now,
                                        filtered_state_covariance = P_now,
                                        observation = measurement)

        # print(x_now)
        # print(measurement)
        print("Time to update kf3: %s seconds" % (time.time() - time_before))
        x_new[i, :] = x_now
        print(x_new)
        i = i + 1

    plt.figure(3)
    old_times = range(measurements.shape[0] - n_real_time)
    new_times = range(measurements.shape[0]-n_real_time, measurements.shape[0])
    plt.plot(times, measurements[:, 0], 'bo',
            times, measurements[:, 1], 'ro',
            old_times, filtered_state_means[:, 0], 'b--',
            old_times, filtered_state_means[:, 2], 'r--',
            new_times, x_new[:, 0], 'b-',
            new_times, x_new[:, 2], 'r-')

    plt.show()

if __name__ == "__main__":
    measurements = np.asarray([(399,293),(403,299),(409,308),(416,315),(418,318),(420,323),(429,326),(423,328),(429,334),(431,337),(433,342),(434,352),(434,349),(433,350),(431,350),(430,349),(428,347),(427,345),(425,341),(429,338),(431,328),(410,313),(406,306),(402,299),(397,291),(391,294),(376,270),(372,272),(351,248),(336,244),(327,236),(307,220)])
    initial_state_mean = [measurements[0, 0],
                      0,
                      measurements[0, 1],
                      0]
    transition_matrix = [[1, 1, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 1],
                     [0, 0, 0, 1]]
    observation_matrix = [[1, 0, 0, 0],
                      [0, 0, 1, 0]]

    times = range(measurements.shape[0])
    time_before = time.time()
    n_real_time = 30

    kalman()

    # ukf = UnscentedKalmanFilter(f, g, initial_state_mean = initial_state_mean)
    # (filtered_state_means, filtered_state_covariances) = ukf.filter(measurements[:, :])