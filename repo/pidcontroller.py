class PIDController:
    """This is a parent class for creating PID controllers using this class
        as a parent, it'll create three degrees-of-freedom PID controller"""

    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.error_integral = 0
        self.error_derivative = 0
        self.dt = 0.01
        self.prev_error = 0


class PIDy(PIDController):
    def __init__(self, kp, ki, kd, setpoint):
        super().__init__(kp, ki, kd, setpoint)

    def update(self, current_state, setpoint):
        self.setpoint = setpoint
        error = ((self.setpoint - current_state[0]) * 0.5 + (0 - current_state[1]) * 0.5)
        self.error_integral = self.error_integral + error * self.dt
        self.error_derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        output = (self.kp * error
                  + self.ki * self.error_integral
                  + self.kd * self.error_derivative)

        return output


class PIDtheta(PIDController):
    def __init__(self, kp, ki, kd, setpoint):
        super().__init__(kp, ki, kd, setpoint)

    def update(self, current_state, setpoint):
        self.setpoint = setpoint
        error = (self.setpoint - current_state[0])
        self.error_integral = self.error_integral + error * self.dt
        self.error_derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        output = (self.kp * error
                  + self.ki * self.error_integral
                  + self.kd * self.error_derivative)

        return output
