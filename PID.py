class controller:
  def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral_sum = 0.0
        self.previous_error = 0.0
        self.setpoint = 0.0
        self.deadzone_thresh = 0.0

    def set_setpoint(self, target):
        self.setpoint = target

    def set_deadzone(self, threshold):
        self.deadzone_thresh = threshold

    def compute(self, current_state, dt):
        error = self.setpoint - current_state

        if abs(error) < self.deadzone_thresh:
            self.previous_error = error
            return 0.0

        P = self.Kp * error

        self.integral_sum += error * dt

        I = self.Ki * self.integral_sum

        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0

        D = self.Kd * derivative

        output = P + I + D
        self.previous_error = error

        return output

def main():
  robot1 = controller(1.5, 0.01, 0.1)
  robot1.set_setpoint(100.0)
  robot1.set_deadzone(0.01)
  current_state = 0.0

  for i in range(50):
      error_before_step = robot1.setpoint - current_state
      output = robot1.compute(current_state, dt=0.1)
      current_state += output * 0.1

      print(
          f"Loop #{i} - Error: {error_before_step:.2f}  "
          f"Output: {output:.2f}  State: {current_state:.2f}"
      )


if __name__ == "__main__":
  main()
