import time

class LowPassFilter:
    def __init__(self, time_constant, prev_time):
        """
        Initialize the low pass filter with the given time constant.
        """
        self.Tf = time_constant  # Low pass filter time constant
        self.y_prev = 0.0  # Previous filtered value
        self.timestamp_prev = prev_time  # Previous timestamp in microseconds

    def __call__(self, x, cur_time):
        """
        Apply the low pass filter to the input signal x.
        """
        timestamp = cur_time  # Get current timestamp
        dt = (timestamp - self.timestamp_prev) * 1e-6  # Time difference in seconds

        # Handle cases where dt is negative or too large
        if dt < 0.0:
            dt = 1e-3
        elif dt > 0.3:
            self.y_prev = x
            self.timestamp_prev = timestamp
            return x

        # Calculate the filter coefficient
        alpha = self.Tf / (self.Tf + dt)

        # Apply the filter
        y = alpha * self.y_prev + (1.0 - alpha) * x

        # Update the previous values
        self.y_prev = y
        self.timestamp_prev = timestamp

        return y
