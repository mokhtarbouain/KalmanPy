from __future__ import division
import math
import random

class Vector2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class KalmanFilter2D:
    def __init__(self, initial_position, initial_velocity, process_noise, measurement_noise):
        self.state = initial_position
        self.velocity = initial_velocity
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        self.error_covariance = [[1, 0], [0, 1]]
        self.kalman_gain = [[0, 0], [0, 0]]

    def predict(self, dt):
        self.state.x += self.velocity.x * dt
        self.state.y += self.velocity.y * dt
        self.error_covariance[0][0] += self.process_noise
        self.error_covariance[1][1] += self.process_noise

    def update(self, measurement):
        self.kalman_gain[0][0] = self.error_covariance[0][0] / (self.error_covariance[0][0] + self.measurement_noise)
        self.kalman_gain[1][1] = self.error_covariance[1][1] / (self.error_covariance[1][1] + self.measurement_noise)

        self.state.x += self.kalman_gain[0][0] * (measurement.x - self.state.x)
        self.state.y += self.kalman_gain[1][1] * (measurement.y - self.state.y)

        self.error_covariance[0][0] *= (1 - self.kalman_gain[0][0])
        self.error_covariance[1][1] *= (1 - self.kalman_gain[1][1])

    def get_state(self):
        return self.state

def main():
    initial_position = Vector2D(0, 0)
    initial_velocity = Vector2D(1, 1)
    process_noise = 1e-3
    measurement_noise = 1e-2
    dt = 1.0

    kf = KalmanFilter2D(initial_position, initial_velocity, process_noise, measurement_noise)

    measurements = [
        Vector2D(1.1, 0.9),
        Vector2D(2.0, 2.1),
        Vector2D(3.0, 3.1),
        Vector2D(4.1, 4.0),
        Vector2D(5.0, 5.2)
    ]

    for measurement in measurements:
        kf.predict(dt)
        kf.update(measurement)
        state = kf.get_state()
        print "Position estimee: (%.2f, %.2f)" % (state.x, state.y)

if __name__ == "__main__":
    main()
