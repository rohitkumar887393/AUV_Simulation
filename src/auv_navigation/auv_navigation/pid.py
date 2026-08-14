# Reusable PID class for AUV control loops

class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, dt=0.05, integral_limit=None, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral_limit = integral_limit
        self.output_limit = output_limit

        self.integral = 0.0
        self.previous_error = 0.0
        self.first_run = True

    def update(self, setpoint, measurement):
        error = setpoint - measurement

        # Proportional term
        p_out = self.kp * error

        # Integral term
        self.integral += error * self.dt
        if self.integral_limit is not None:
            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        i_out = self.ki * self.integral

        # Derivative term
        if self.first_run:
            d_out = 0.0
            self.first_run = False
        else:
            derivative = (error - self.previous_error) / self.dt
            d_out = self.kd * derivative

        self.previous_error = error

        # Combined output
        output = p_out + i_out + d_out

        # Output clamping
        if self.output_limit is not None:
            output = max(min(output, self.output_limit), -self.output_limit)

        return output

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.first_run = True
