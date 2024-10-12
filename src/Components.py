from neopixel import NeoPixel
from machine import I2C, PWM
from time import ticks_us, ticks_diff
from machine import time_pulse_us

class Light:
    def __init__(self, np: NeoPixel, index: int):
        self.is_turned_on = False
        self.is_blinking = False
        self.is_currently_on = False
        self.np = np
        self.index = index

    def turn_on(self, color: tuple, blinking: bool):
        self.is_on = True
        self.is_currently_on = True
        self.is_blinking = blinking
        self.color = color
        self.np[self.index] = color

    def turn_off(self):
        self.is_on = False
        self.is_currently_on = False
        self.is_blinking = False
        self.np[self.index] = (0, 0, 0)

    def process(self):
        # If blinking, revert state
        if self.is_blinking:
            if self.is_currently_on:
                self.np[self.index] = (0, 0, 0)
                self.is_currently_on = False
            else:
                self.np[self.index] = self.color
                self.is_currently_on = True

class Motor:
    def __init__(self, forward_pwm_pin: PWM, backward_pwm_pin: PWM):
        self.forward_pwm_pin = forward_pwm_pin
        self.backward_pwm_pin = backward_pwm_pin
        self.forward(0)

    def forward(self, pwm: int):
        self.forward_pwm_pin.duty_u16(pwm)
        self.backward_pwm_pin.duty_u16(0)

    def backward(self, pwm: int):
        self.forward_pwm_pin.duty_u16(0)
        self.backward_pwm_pin.duty_u16(pwm)

class SpeedSensor:
    MIN_TIME_BETWEEN_EVENTS_US = 2000
    SPEED_HISTORY_DEPTH = 8
    MEDIAN_FILTER_WINDOW_SIZE = 3
    SPEED_THRESHOLD = 1

    def __init__(self, zero_speed_threshold: int, ticks_per_round: int):
        self.zero_speed_threshold = zero_speed_threshold
        self.last_change_time = ticks_us()
        self.last_state = 0
        self.speed_history = [0] * self.SPEED_HISTORY_DEPTH
        self.radians_per_tick = 2 * 3.14159265359 / ticks_per_round 
        self.ticks_counter = 0
        self.radians_counter = 0.0

    def process_event(self, pin=None):
        # React only if state is different than previous one
        if pin.value() != self.last_state:
            # Store new state
            self.last_state = pin.value()

            # Increment counter
            self.ticks_counter += 1
            self.radians_counter += self.radians_per_tick
            
            # Measure time between events
            time_between_events = ticks_diff(ticks_us(), self.last_change_time)

            # Check if the time between events is within the expected range
            if time_between_events > self.MIN_TIME_BETWEEN_EVENTS_US:
                # Insert new value to the beginning of the list and remove the last one
                self.speed_history.insert(0, time_between_events)
                self.speed_history.pop()
                
                # Store the time of the change
                self.last_change_time = ticks_us()

    # This function is used to smooth extreme outliers from the speed history
    def median_filter(self, data, window_size):
        filtered_data = []
        half_window = window_size // 2
        for i in range(len(data)):
            start = max(0, i - half_window)
            end = min(len(data), i + half_window + 1)
            window = data[start:end]
            median = sorted(window)[len(window) // 2]
            filtered_data.append(median)
        return filtered_data

    # This function returns rolling average speed in radians per second
    def get_speed_radians(self) -> float:
        # Get cleaned up history
        filtered_history = self.median_filter(self.speed_history, self.MEDIAN_FILTER_WINDOW_SIZE)

        # Make sure we are not dividing by zero
        if sum(filtered_history) != 0:
            rolling_avg = sum(filtered_history) / len(filtered_history)
            # If time difference between now and last tick is much higher than the last tick
            # and rolling average, we are quickly slowing down - return time from list tick only
            time_from_last_tick = ticks_diff(ticks_us(), self.last_change_time)
            if time_from_last_tick > 2*filtered_history[0] and time_from_last_tick > 2*rolling_avg:
                rolling_avg = time_from_last_tick

            # If speed is very low, return 0
            speed_radians_per_second = self.radians_per_tick / rolling_avg * 1000000
            if speed_radians_per_second < self.SPEED_THRESHOLD:
                return 0
            else:
                return speed_radians_per_second
        else:
            return 0

class UltrasonicSensor:
    SOUND_SPEED = 340

    def __init__(self, trigger_pin, echo_pin):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin

    def measure_distance(self) -> float:
        self.trigger_pin.value(1)
        self.trigger_pin.value(0)
        echo_delay_us = time_pulse_us(self.echo_pin, 1)
        distance_m = ((echo_delay_us / 1000000)*self.SOUND_SPEED)/2
        return distance_m


class CollisionSensor:
    def __init__(self, hw_address):
        self.hw_address = hw_address

    def is_collision(self) -> bool:
        pass  # Implement the logic to detect collision


class TrackingSensors:
    def __init__(self, i2c: I2C, hw_address):
        self.i2c = i2c
        self.hw_address = hw_address

    def is_on_line(self):
        raw_data = self.i2c.readfrom(self.hw_address, 1)
        raw_data_int = int.from_bytes(raw_data, 'big')
        binary_string = '{:08b}'.format(raw_data_int)
        bit_array = [int(bit) for bit in binary_string]
        output = {
            "left": bit_array[5] == 1,    # 1 means on black line so return True
            "center": bit_array[4] == 1,
            "right": bit_array[3] == 1
        }
        return output
    
