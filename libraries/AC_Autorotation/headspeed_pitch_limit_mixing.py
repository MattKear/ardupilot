import numpy as np
from matplotlib import pyplot as plt
import math

def is_positive(value):
    return value > 0

def constrain_float(value, min_val, max_val):
    return max(min_val, min(value, max_val))



class APMotorsHeliDual:
    def __init__(self):
        self.limit = {}
        self.limit['throttle_lower'] = False
        self.limit['throttle_upper'] = False
        self.limit['yaw'] = False
        self.limit['pitch'] = False
        self.limit['roll'] = False

        self._dual_mode = 'TANDEM'
        self._cyclic_max = 2500

        self._heliflags = {}
        self._heliflags['landing_collective'] = False

        self._yaw_rev_expo = 0
        self._yaw_scaler = 0.25
        self._dcp_yaw_effect = 0

        self._dcp_trim = 0
        self._dcp_scaler = 0.25

        self._servo_mode = "AUTOMATED"

        self._collective_max = 1750
        self._collective_min = 1250
        self._collective2_max = 1750
        self._collective2_min = 1250

        self.col_angle_min = -6
        self.col_angle_max = 12
        col_zero_thrust = 0
        land_angle_min = -2

        self._collective_land_min_pct = self.col_pct_from_angle(land_angle_min)
        print("Landed Col Percent = %.2f \n" % self._collective_land_min_pct)

        self._collective_zero_thrst_pct = self.col_pct_from_angle(col_zero_thrust)
        self._collective2_zero_thrst_pct = self.col_pct_from_angle(col_zero_thrust)
        print("Zero Thrust Percent = %.2f \n" % self._collective_zero_thrst_pct)

        self.head_speed_constrain = False

        self.pitch1_out = 0
        self.pitch2_out = 0
        self.roll1_out = 0
        self.roll2_out = 0
        self.col1_out = 0
        self.col2_out = 0

        self.last_col1 = 0
        self.last_col2 = 0

    def col_pct_from_angle(self, ang):
        return (ang - self.col_angle_min) / (self.col_angle_max - self.col_angle_min)

    def move_actuators(self, roll_out, pitch_out, collective_in, yaw_out):
        # Initialize limits
        self.limit['throttle_lower'] = False
        self.limit['throttle_upper'] = False

        cyclic_limit = self._cyclic_max / 4500.0

        if self._dual_mode in ['TRANSVERSE', 'INTERMESHING']:
            if pitch_out < -cyclic_limit:
                pitch_out = -cyclic_limit
                self.limit['pitch'] = True
            if pitch_out > cyclic_limit:
                pitch_out = cyclic_limit
                self.limit['pitch'] = True

        if self._dual_mode != 'TRANSVERSE':
            if roll_out < -cyclic_limit:
                roll_out = -cyclic_limit
                self.limit['roll'] = True
            if roll_out > cyclic_limit:
                roll_out = cyclic_limit
                self.limit['roll'] = True

        # Constrain collective input
        collective_out = collective_in
        if collective_out <= 0.0:
            collective_out = 0.0
            self.limit['throttle_lower'] = True
        if collective_out >= 1.0:
            collective_out = 1.0
            self.limit['throttle_upper'] = True

        if self._heliflags['landing_collective'] and collective_out < self._collective_land_min_pct:
            collective_out = self._collective_land_min_pct
            self.limit['throttle_lower'] = True

        self._heliflags['below_land_min_coll'] = not is_positive(collective_out - self._collective_land_min_pct)

        self.update_takeoff_collective_flag(collective_out)

        # Set rear collective
        collective2_out = collective_out
        if self._servo_mode == 'MANUAL_CENTER':
            collective2_out = self._collective2_zero_thrst_pct

        # Apply pre-compensation for yaw if needed
        if self._servo_mode == 'AUTOMATED':
            yaw_compensation = 0.0
            if self._dual_mode == 'INTERMESHING':
                if self._yaw_rev_expo > 0.01:
                    exponent = self._yaw_rev_expo * (collective_out - self._collective_zero_thrust_pct)
                    yaw_compensation = 1.0 - (2.0 / (1.0 + math.exp(exponent)))
                    yaw_out *= yaw_compensation
            else:
                if self._dual_mode == 'TRANSVERSE':
                    yaw_compensation = self._dcp_yaw_effect * roll_out
                else:  # TANDEM
                    yaw_compensation = self._dcp_yaw_effect * pitch_out
                yaw_out += yaw_compensation

        # Scale yaw
        if yaw_out < -cyclic_limit:
            yaw_out = -cyclic_limit
            self.limit['yaw'] = True
        if yaw_out > cyclic_limit:
            yaw_out = cyclic_limit
            self.limit['yaw'] = True

        # Scale collective for swashplate
        collective_scaler = (self._collective_max - self._collective_min) * 0.001
        collective_out_scaled = collective_out * collective_scaler + (self._collective_min - 1000) * 0.001

        collective2_scaler = (self._collective2_max - self._collective2_min) * 0.001
        collective2_out_scaled = collective2_out * collective2_scaler + (self._collective2_min - 1000) * 0.001

        # Feed power estimate to main rotor (for throttle curve)
        # self._main_rotor.set_collective(abs(collective_out))

        # Mix swash plate
        if self._dual_mode == 'TANDEM':
            self.mix_tandem(pitch_out, roll_out, yaw_out, collective_out_scaled, collective2_out_scaled)
        elif self._dual_mode == 'TRANSVERSE':
            self.mix_transverse(pitch_out, roll_out, yaw_out, collective_out_scaled, collective2_out_scaled)
        elif self._dual_mode == 'INTERMESHING':
            self.mix_intermeshing(pitch_out, roll_out, yaw_out, collective_out_scaled, collective2_out_scaled)
        else:
            self.mix_tandem(pitch_out, roll_out, yaw_out, collective_out_scaled, collective2_out_scaled)


    def mix_tandem(self, pitch_input, roll_input, yaw_input, collective1_input, collective2_input):
        # Differential cyclic roll is used for yaw and combined for roll
        swash1_roll = roll_input + self._yaw_scaler * yaw_input
        swash2_roll = roll_input - self._yaw_scaler * yaw_input

        # Handle combined control limiting for roll and yaw
        if abs(swash1_roll) > 1.0:
            swash1_roll = constrain_float(swash1_roll, -1.0, 1.0)
            self.limit['roll'] = True
            self.limit['yaw'] = True
        if abs(swash2_roll) > 1.0:
            swash2_roll = constrain_float(swash2_roll, -1.0, 1.0)
            self.limit['roll'] = True
            self.limit['yaw'] = True

        # Cyclic is not used for pitch control
        swash_pitch = 0.0

        # Differential collective for pitch and combined for thrust
        dcp_trim_limited = constrain_float(self._dcp_trim, -0.2, 0.2)
        swash1_coll =  0.45 * self._dcp_scaler * (pitch_input + dcp_trim_limited) + collective1_input
        swash2_coll = -0.45 * self._dcp_scaler * (pitch_input + dcp_trim_limited) + collective2_input



        # we may have been told to manage head speed by rescaling the collective output such that the
        # slowest head does not move the collective any higher
        if (self.head_speed_constrain):
            # Find the slower head, this will be the one with the higher collective
            delta_col = 0
            if (swash1_coll > swash2_coll):
                # Swash 1 has the higher collective
                delta_col = swash1_coll - self.last_col1

            else:
                # Swash 2 has the higher collective
                delta_col = swash2_coll - self.last_col2
            # NOTE: we can ask the swashplates for the last collective values as these are stored for logging anyway

            # Apply the delta to both cols
            swash1_coll = swash1_coll + delta_col
            swash2_coll = swash2_coll + delta_col
        else:
            self.last_col1 = self.col1_out
            self.last_col2 = self.col2_out


        # Handle combined control limiting
        self.limit['pitch'] |= self.constrain_collective(swash1_coll)
        self.limit['pitch'] |= self.constrain_collective(swash2_coll)

        self.pitch1_out = swash_pitch
        self.pitch2_out = swash_pitch
        self.roll1_out = swash1_roll
        self.roll2_out = swash2_roll
        self.col1_out = swash1_coll
        self.col2_out = swash2_coll

        # Calculate servo positions
        # self._swashplate1.calculate(swash1_roll, swash_pitch, swash1_coll)
        # self._swashplate2.calculate(swash2_roll, swash_pitch, swash2_coll)

    def mix_transverse(self, pitch, roll, yaw, coll1, coll2):
        pass

    def mix_intermeshing(self, pitch, roll, yaw, coll1, coll2):
        pass

    def constrain_collective(self, col):
        if (col > 1.0):
            col = 1.0
            self.limit["throttle_upper"] = True
            return True

        if (col < -1.0):
            col = -1.0
            self.limit["throttle_lower"] = True
            return True
        
        # If we got this far, nothing is being limited
        return False

    def update_takeoff_collective_flag(self, collective_out):
        pass


    def set_head_speed_constrain(self, val):
        self.head_speed_constrain = val














if __name__ == "__main__":

    n_timesteps = 100

    time = np.linspace(0, 5, n_timesteps)

    col = np.linspace(0.25, 0.25, n_timesteps)
    pitch = np.linspace(0, -1, n_timesteps) # pitch forward maneuver
    roll = 0
    yaw_out = 0

    motors = APMotorsHeliDual()

    col1_mix_out = []
    col2_mix_out = []
    pitch1_mix_out = []
    pitch2_mix_out = []
    pitch_last = 0
    for c,p,t in zip(col, pitch, time):

        if (t > 2.0):
            # Simulate a headspeed reaching min target speed
            motors.set_head_speed_constrain(True)
            # p = pitch_last

        if (t > 4.0):
            motors.set_head_speed_constrain(False)

        motors.move_actuators(roll, p, c, yaw_out)

        col1_mix_out.append(motors.col1_out)
        col2_mix_out.append(motors.col2_out)
        pitch1_mix_out.append(motors.pitch1_out)
        pitch2_mix_out.append(motors.pitch2_out)

        pitch_last = p


    fig, ax = plt.subplots()
    ax.plot(time, col1_mix_out, label="Col 1")
    ax.plot(time, col2_mix_out, label="Col 2")
    # ax.plot(time, pitch1_mix_out, label="Pitch 1")
    # ax.plot(time, pitch2_mix_out, label="Pitch 2")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Output to Swashplate Mixer")
    plt.legend()

    plt.show()


