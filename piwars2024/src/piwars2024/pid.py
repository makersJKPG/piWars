

class pid:

    def __init__(self, Kp, Ki, Kd, T, tau = 0.0, limMin = -1.0, limMax = 1.0, limMinInt = -1.0, limMaxInt = 1.0):

            # Controller gains
            self.Kp = Kp
            self.Ki = Ki
            self.Kd = Kd

            # Sample time (in seconds)
            self.T = T

            # Derivative low-pass filter time constant
            self.tau = tau

            # Output limits
            self.limMin = limMin
            self.limMax = limMax

            # Integrator limits
            self.limMinInt = limMinInt
            self.limMaxInt = limMaxInt

            # Controller "memory"
            self.integrator = 0.0
            self.prevError = 0.0	# Required for integrator
            self.differentiator = 0.0
            self.prevMeasurement = 0.0	# Required for differentiator

            # Controller output
            self.out = 0.0

    def update(self, setpoint, measurement):

        # Error signal
        error = setpoint - measurement

        # Proportional
        proportional = self.Kp * error

        # Integral
        self.integrator = self.integrator + 0.5 * self.Ki * self.T * (error + self.prevError)

        # Anti-wind-up via integrator clamping
        if self.integrator > self.limMaxInt:
            self.integrator = self.limMaxInt
        elif self.integrator < self.limMinInt:
            self.integrator = self.limMinInt

        # Derivative (band-limited differentiator)
	# Note: derivative on measurement, therefore minus sign in front of equation!
        self.differentiator = -(2.0 * self.Kd * (measurement - self.prevMeasurement) + (2.0 * self.tau - self.T) * self.differentiator) / (2.0 * self.tau + self.T)

        # Compute output and apply limits
        self.out = proportional + self.integrator + self.differentiator

        if self.out > self.limMax:
            self.out = self.limMax
        elif self.out < self.limMin:
            self.out = self.limMin

        # Store error and measurement for later use
        self.prevError = error
        self.prevMeasurement = measurement

        # Return controller output
        return self.out
