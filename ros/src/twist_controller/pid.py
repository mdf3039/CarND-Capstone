from collections import deque

MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx
        self.d1 = deque(maxlen=8)

        self.int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0

    def step(self, error, sample_time, kp, ki, kd):

        # integral = self.int_val + error * sample_time;
        self.d1.append(error*sample_time)
        derivative = (error - self.last_error) / sample_time;

        y = kp * error + ki * np.sum(self.d1,axis=0) + kd * derivative;
        val = y # max(self.min, min(y, self.max))

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        # else:
            # self.int_val = integral
        self.last_error = error

        return val
