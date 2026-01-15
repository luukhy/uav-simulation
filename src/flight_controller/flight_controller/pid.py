class PID:
    def __init__(self, kp, ki, kd, i_max=100.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_max = i_max
        
        self.integral = 0.0
        self.prev_input = 0.0  
        self.first_run = True

    def calculate(self, target, current, dt) -> float:
        if dt <= 0.0: return 0.0

        error = target - current
        p_term = self.kp * error

        self.integral += error * dt
        self.integral = max(min(self.integral, self.i_max), -self.i_max)
        i_term = self.ki * self.integral

        if self.first_run:
            self.prev_input = current
            self.first_run = False
            
        derivative = (current - self.prev_input) / dt
        d_term = -1.0 * self.kd * derivative 

        self.prev_input = current

        return p_term + i_term + d_term