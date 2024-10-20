from Components import Motor, SpeedSensor
from machine import I2C, Pin, ADC, PWM
from utime import sleep_ms
from random import randint
from Configuration import CONFIG

class Direction:
    FORWARD = "forward"
    BACKWARD = "backward"

class TurnDirection:
    CLOCKWISE = "clockwise"
    COUNTER_CLOCKWISE = "counter_clockwise"

class MotorsController():
    """
    Class that handles the motor control of the robot.
    This includes speed to PWM calculations, calibration and speed regulation based on sensor feedback.
    """

    def __init__(self, left_motor_forward_pwm_pin: PWM, left_motor_backward_pwm_pin: PWM, right_motor_forward_pwm_pin: PWM, right_motor_backward_pwm_pin: PWM):
        # Instantiate the motors and speed sensors
        self.left_motor = Motor(left_motor_forward_pwm_pin, left_motor_backward_pwm_pin)
        self.right_motor = Motor(right_motor_forward_pwm_pin, right_motor_backward_pwm_pin)
        self.left_speed_sensor = SpeedSensor(0, CONFIG["WHEEL_HOLES"]*2)
        self.right_speed_sensor = SpeedSensor(0, CONFIG["WHEEL_HOLES"]*2)

        # Initialize the calibration parameters
        self.slope = CONFIG["CALIBRATION_DEFAULT_SLOPE"]
        self.intercept = CONFIG["CALIBRATION_DEFAULT_INTERCEPT"]

        # Initialize variables
        self.desired_speed_radians = 0
        self.right_desired_speed = 0
        self.left_desired_speed = 0
        self.right_desired_speed_radians = 0
        self.left_desired_speed_radians = 0
        self.left_pwm = 0
        self.right_pwm = 0

        # Initialize the desired direction
        self.desired_direction = Direction.FORWARD

    def drive(self, speed: float, direction: Direction, angular_velocity: float = 0):
        """
        Actual driving functionality that takes speed in m/s and direction as input.
        It also optionally takes angular velocity for curved drive with positive angular velocity for left turn and negative for right turn.
        This might be called by movement controller to change actual speed 
        when desired cannot be achieved due to collision avoidance or adaptive cruise control.
        """
        self.desired_speed_radians = speed / (CONFIG["WHEEL_DIAMETER"] / 2)
        self.right_desired_speed = speed - (CONFIG["WHEEL_BASE_DISTANCE"] / 2) * angular_velocity
        self.left_desired_speed = speed + (CONFIG["WHEEL_BASE_DISTANCE"] / 2) * angular_velocity
        self.right_desired_speed_radians = self.right_desired_speed / (CONFIG["WHEEL_DIAMETER"] / 2)
        self.left_desired_speed_radians = self.left_desired_speed / (CONFIG["WHEEL_DIAMETER"] / 2)
        right_predicted_pwm = self.predict_pwm(self.right_desired_speed_radians)
        left_predicted_pwm = self.predict_pwm(self.left_desired_speed_radians)

        self.left_pwm = left_predicted_pwm
        self.right_pwm = right_predicted_pwm

        # Drive the motors
        if direction == Direction.FORWARD:
            self.right_motor.forward(right_predicted_pwm)
            self.left_motor.forward(left_predicted_pwm)
        elif direction == Direction.BACKWARD:
            self.right_motor.backward(right_predicted_pwm)
            self.left_motor.backward(left_predicted_pwm)

        # print(f"Speed: {speed}, desired_speed_radians: {self.desired_speed_radians}, angular_velocity: {angular_velocity}, left_desired_speed: {self.left_desired_speed}, right_desired_speed: {self.right_desired_speed}")
        # print(f"left_pwm: {self.left_pwm}, right_pwm: {self.right_pwm}")
        # print(f"left_desired_speed_radians: {self.left_desired_speed_radians}, right_desired_speed_radians: {self.right_desired_speed_radians}")

    def calibrate(self, motor: Motor, speed_sensor: SpeedSensor, sleep = 1000, step_size = 128):
        """
        Calibrate the motor by measuring the speed for different PWM values.
        Note there are default values for the calibration parameters so running this always is not mandatory.
        Calibrated values get stored in memory - after restart old defaults will be loaded.
        """
        print("Calibrating motor...")
        radians_values = []
        for pwm in range(0, 65536, step_size):
            motor.forward(pwm)
            sleep_ms(sleep)
            radians_values.append(speed_sensor.get_speed_radians())
            print(f"Speed for pwm {pwm}: {radians_values[-1]}")
        motor.forward(0)
        self.create_linear_model(radians_values, step_size)

    # def calibrate_advanced(self, motor: Motor, speed_sensor: SpeedSensor, file_name):
    #     """
    #     Gather data for advanced calibration and store it in a file.
    #     Measure steady state voltage of the system with each PWM value and speed it produces.
    #     To get variety of data points we randomize PWM rather than going sequentially.
    #     Steady voltage (no motors running) reading and PWM to speed values are stored in a file.
    #     File is used outside of device to calculate ML model.
    #     """
    #     print("Calibrating motor...")
    #     voltage_pin = ADC(Pin(self.VOLTAGE_PIN_NUMBER, Pin.IN))
    #     with open(file_name, "w") as file:
    #         file.write(f"pwm,voltage,speed\n")
    #         while True:   # Do until batteries die
    #             voltage = 3.3 * voltage_pin.read_u16() / 65535
    #             pwm = randint(0, 255)
    #             motor.forward(pwm)
    #             sleep_ms(3000)
    #             speed = speed_sensor.get_speed_radians()
    #             print(f"pwm: {pwm}, voltage: {voltage}, speed: {speed}")
    #             file.write(f"{pwm},{voltage},{speed}\n")
    #             motor.forward(0)
    #             sleep_ms(200)

    def create_linear_model(self, radians_values, step_size):
        """"
        Create a linear model for the motor calibration.
        Note we are using single model for both motors at the moment.
        """
        pwm_values = list(range(0, 65536, step_size))
        
        # Calculate the means
        mean_radians = sum(radians_values) / len(radians_values)
        mean_pwm = sum(pwm_values) / len(pwm_values)
        
        # Calculate the slope
        numerator = sum((radians - mean_radians) * (pwm - mean_pwm) for radians, pwm in zip(radians_values, pwm_values))
        denominator = sum((radians - mean_radians) ** 2 for radians in radians_values)
        self.slope = numerator / denominator
        
        # Calculate the intercept
        self.intercept = mean_pwm - self.slope * mean_radians
        
        print(f"len(radians_values): {len(radians_values)}")
        print(f"len(pwm_values): {len(pwm_values)}")
        print(f"mean_radians: {mean_radians}")
        print(f"mean_pwm: {mean_pwm}")
        print(f"Linear model parameters: slope = {self.slope}, intercept = {self.intercept}")

    def predict_pwm(self, radians):
        """
        Predict the PWM value for a given speed in radians per second.
        """
        pwm_value = round(self.slope * radians + self.intercept)
        if pwm_value < CONFIG["MIN_DRIVE_PWM"] or radians == 0:
            return 0
        elif pwm_value > 65535:
            return 65535
        return pwm_value
    
    def speed_regulation(self, desired_direction=None, timer=None):
        """
        Regulate speed by adjusting PWM of motors based on measured feedback.
        """
        self.right_pwm = self.speed_regulation_execution(
            desired_direction=desired_direction, 
            desired_speed_radians=self.right_desired_speed_radians,
            speed_sensor=self.right_speed_sensor, 
            motor=self.right_motor, 
            pwm=self.right_pwm)
        self.left_pwm = self.speed_regulation_execution(
            desired_direction=desired_direction, 
            desired_speed_radians=self.left_desired_speed_radians,
            speed_sensor=self.left_speed_sensor, 
            motor=self.left_motor, 
            pwm=self.left_pwm)

    def speed_regulation_execution(self, desired_direction: Direction, desired_speed_radians, speed_sensor: SpeedSensor, motor: Motor, pwm):
        """
        Perform the speed regulation control action on specific motor.
        """
        # Get current speed
        speed = speed_sensor.get_speed_radians()
        error = desired_speed_radians - speed

        # Calculate the control action and update the PWM
        control_action = round(CONFIG["SPEED_REGULATION_P"] * error)
        pwm = pwm + control_action

        # Cap the PWM value at 65535
        pwm = max(min(pwm, 65535), -65535)

        # If the PWM is negative, invert the direction
        if pwm < 0:
            abs_pwm = abs(pwm)
            if desired_direction == Direction.FORWARD:
                motor.backward(abs_pwm)
            else:
                motor.forward(abs_pwm)
        # Otherwise, drive the motors in the desired direction
        else:
            if desired_direction == Direction.FORWARD:
                motor.forward(pwm)
            else:
                motor.backward(pwm)

        # print(f"Speed: {speed}, Desired speed: {desired_speed_radians}, Error: {error}, Control action: {control_action}, PWM: {pwm}")

        return pwm

    def stop(self):
        self.right_motor.backward(0)
        self.left_motor.backward(0)
        self.desired_speed_radians = 0
        self.left_pwm = 0
        self.right_pwm = 0

    def stationary_turn(self, speed: float, turn_direction: TurnDirection):
        """
        Perform a stationary turn.
        """
        self.desired_speed_radians = speed / (CONFIG["WHEEL_DIAMETER"] / 2)
        right_predicted_pwm = self.predict_pwm(self.desired_speed_radians)
        left_predicted_pwm = self.predict_pwm(self.desired_speed_radians)

        if turn_direction == TurnDirection.COUNTER_CLOCKWISE:
            self.right_motor.forward(right_predicted_pwm)
            self.left_motor.backward(left_predicted_pwm)
        elif turn_direction == TurnDirection.CLOCKWISE:
            self.right_motor.backward(right_predicted_pwm)
            self.left_motor.forward(left_predicted_pwm)

    def get_robot_speed(self):
        pass
