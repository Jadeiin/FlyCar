def constrain(value, min_value, max_value):
    return max(min(value, max_value), min_value)


class PIDController:
    """
    PID controller class in Python
    """

    def __init__(self, P, I, D, ramp, limit, prev_time):
        """
        Initialize the PID controller.

        :param P: Proportional gain
        :param I: Integral gain
        :param D: Derivative gain
        :param ramp: Maximum speed of change of the output value
        :param limit: Maximum output value
        """
        self.P = P
        self.I = I
        self.D = D
        self.output_ramp = ramp  # Output derivative limit [units/second]
        self.limit = limit  # Output supply limit [units]

        self.error_prev = 0.0
        self.output_prev = 0.0
        self.integral_prev = 0.0
        self.timestamp_prev = prev_time

    def __call__(self, error, cur_time):
        """
        Calculate the PID control output based on the error.

        :param error: The error value (setpoint - measured value)
        :return: PID output
        """
        # Calculate time elapsed since last call
        timestamp_now = cur_time
        Ts = (timestamp_now - self.timestamp_prev) * 1e-6
        # Handle micros overflow or invalid time intervals
        if Ts <= 0 or Ts > 0.5:
            Ts = 1e-3

        # Proportional term
        proportional = self.P * error

        # Integral term using Tustin transform
        integral = self.integral_prev + self.I * Ts * 0.5 * (error + self.error_prev)
        integral = constrain(integral, -self.limit, self.limit)

        # Derivative term
        derivative = self.D * (error - self.error_prev) / Ts

        # Combine all terms
        output = proportional + integral + derivative
        output = constrain(output, -self.limit, self.limit)

        # Apply ramping if defined
        if self.output_ramp > 0:
            output_rate = (output - self.output_prev) / Ts
            if output_rate > self.output_ramp:
                output = self.output_prev + self.output_ramp * Ts
            elif output_rate < -self.output_ramp:
                output = self.output_prev - self.output_ramp * Ts

        # Store previous values for the next iteration
        self.integral_prev = integral
        self.output_prev = output
        self.error_prev = error
        self.timestamp_prev = timestamp_now

        return output

    def reset(self):
        """
        Reset the PID controller's internal state.
        """
        self.integral_prev = 0.0
        self.output_prev = 0.0
        self.error_prev = 0.0
