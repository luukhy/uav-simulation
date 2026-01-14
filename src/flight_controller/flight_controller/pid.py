import time

class PID:
    def __init__(self, kp, ki, kd, i_max=100.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_max = i_max # anti-windup limit
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = time.time()

    def calculate(self, target, current):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0.0: return 0.0 

        error = target - current

        p_term = self.kp * error

        self.integral += error * dt
        self.integral = max(min(self.integral, self.i_max), -self.i_max)
        i_term = self.ki * self.integral

        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative

        self.prev_error = error
        self.last_time = current_time

        return p_term + i_term + d_term