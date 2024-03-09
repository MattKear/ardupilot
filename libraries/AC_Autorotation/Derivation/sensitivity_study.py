# Plot out the sensitivities of all of the inputs to the flare height calculation

import numpy as np
import math
from matplotlib import pyplot as plt

SSL_AIR_DENSITY = 1.225
ASSUMED_CD0 = 0.011
AP_ALPHA_TPP = 20.0
GRAVITY_MSS = 9.81

def norm(a,b):
    return np.sqrt(a**2 + b**2)

def deg2rad(a):
    return a*np.pi/180

def safe_asin(f):
    if f >= 1.0:
        return 2 * math.pi
    if f <= -1.0:
        return -2 * math.pi
    return math.asin(f)

# =====================================================================================
# Math from Ferruccio's original PR
class Flare_Calc_Pre_Changes():
    def __init__(self):
        self.reset_to_defaults()

    def reset_to_defaults(self):
        self.var = {}
        self.var['_param_solidity'] = 0.05
        self.var['_c_l_alpha'] = np.pi*2
        self.var['_param_diameter'] = 1.25
        self.var['blade_pitch_hover_deg'] = 5.0
        self.var['_param_head_speed_set_point'] = 1500
        self.var['_param_target_speed'] = 1100
        self.var['_t_tch'] = 0.55

    def initial_flare_estimate(self):

        # estimate hover thrust
        # _col_hover_rad = radians(_col_min + (_col_max - _col_min)*_col_hover)
        _col_hover_rad = deg2rad(self.var['blade_pitch_hover_deg'])
        b = self.var['_param_solidity']*6.28
        _disc_area=3.14*(self.var['_param_diameter']*0.5)**2
        lambda_eq = (-(b/8.0) + math.sqrt((b**2)/64.0 +((b/3.0)*_col_hover_rad)))*0.5
        freq=self.var['_param_head_speed_set_point']/60.0
        tip_speed= freq*6.28*self.var['_param_diameter']*0.5
        self._lift_hover = ((1.225*(tip_speed**2)*(self.var['_param_solidity']*_disc_area))*((_col_hover_rad/3.0) - (lambda_eq/2.0))*5.8)*0.5

        # estimate rate of descent
        omega_auto=(self.var['_param_head_speed_set_point']/60.0)*6.28
        tip_speed_auto = omega_auto*self.var['_param_diameter']*0.5
        c_t = self._lift_hover/(0.6125*_disc_area*(tip_speed**2))
        _est_rod=((0.25*(self.var['_param_solidity']*0.011/c_t)*tip_speed_auto)+(((c_t**2)/(self.var['_param_solidity']*0.011))*tip_speed_auto))

        # estimate rotor C_d
        self._c=(self._lift_hover/((_est_rod**2)*0.5*1.225*_disc_area))*1.15
        self._c=min(max(self._c, 0.7), 1.4)

        # calc flare altitude
        des_spd_fwd=self.var['_param_target_speed']*0.01

        return (self.calc_flare_alt(-_est_rod, des_spd_fwd), self._lift_hover/GRAVITY_MSS, lambda_eq)

    def calc_flare_alt(self, sink_rate, fwd_speed):

        #compute speed module and glide path angle during descent
        speed_module=norm(sink_rate,fwd_speed)
        glide_angle=safe_asin(3.14/2-(fwd_speed/speed_module))

        # estimate inflow velocity at beginning of flare
        inflow= -speed_module*math.sin(glide_angle+deg2rad(AP_ALPHA_TPP))

        # estimate flare duration
        k_1=abs((-sink_rate+0.001-math.sqrt(self._lift_hover/self._c))/(-sink_rate+0.001+math.sqrt(self._lift_hover/self._c)))
        k_2=abs((inflow-math.sqrt(self._lift_hover/self._c))/(inflow+math.sqrt(self._lift_hover/self._c)))
        delta_t_flare=(0.5*(self._lift_hover/(9.8065*self._c))*math.sqrt(self._c/self._lift_hover)*math.log(k_1))-(0.5*(self._lift_hover/(9.8065*self._c))*math.sqrt(self._c/self._lift_hover)*math.log(k_2))

        # estimate flare delta altitude
        a=(2*math.sqrt(self._c*9.8065/(self._lift_hover/9.8065)))*delta_t_flare + (2*math.sqrt(self._c*9.8065/(self._lift_hover/9.8065)))*(0.5*(self._lift_hover/(9.8065*self._c))*math.sqrt(self._c/self._lift_hover)*math.log(k_1))
        x=1-math.exp(a)
        s=1-math.exp((2*math.sqrt(self._c*9.8065/(self._lift_hover/9.8065)))*(0.5*(self._lift_hover/(9.8065*self._c))*math.sqrt(self._c/self._lift_hover)*math.log(k_1)))
        d=math.sqrt(self._lift_hover/self._c)
        flare_distance=((2*d/(2*math.sqrt(self._c*9.8065/(self._lift_hover/9.8065))))*(a-math.log(abs(x))-(2*math.sqrt(self._c*9.8065/(self._lift_hover/9.8065)))*(0.5*(self._lift_hover/(9.8065*self._c))*math.sqrt(self._c/self._lift_hover)*math.log(k_1)) +math.log(abs(s))))-d*delta_t_flare
        delta_h= -flare_distance*math.cos(deg2rad(AP_ALPHA_TPP))

        # estimate altitude to begin collective pull
        _cushion_alt = (-(sink_rate*math.cos(deg2rad(AP_ALPHA_TPP)))*self.var['_t_tch'])*100.0

        # total delta altitude to ground
        _flare_alt_calc = _cushion_alt+delta_h*100.0

        return _flare_alt_calc*1e-2
# =====================================================================================

# =====================================================================================
# Math from my branch, after my dicking around
class Flare_Calc():
    def __init__(self):
        self.reset_to_defaults()

    def reset_to_defaults(self):
        self.var = {}
        self.var['_param_solidity'] = 0.05
        self.var['_c_l_alpha'] = np.pi*2
        self.var['_param_diameter'] = 1.25
        self.var['blade_pitch_hover_deg'] = 5.0
        self.var['_param_head_speed_set_point'] = 1500
        self.var['_param_target_speed'] = 1100
        self.var['_t_tch'] = 0.55

    def initial_flare_estimate(self):

        blade_pitch_hover_rad = deg2rad(self.var['blade_pitch_hover_deg'])

        b = self.var['_param_solidity'] * self.var['_c_l_alpha']
        _disc_area = np.pi * 0.25 * self.var['_param_diameter']**2

        # Calculating the equivalent inflow ratio (average across the whole blade)
        # lambda_eq = -b / 16.0 + math.sqrt((b**2) / 256.0 + b * blade_pitch_hover_rad / 8.0)
        # lambda_eq = -b / 16.0 + math.sqrt((b**2) / 256.0 + b * blade_pitch_hover_rad / 12.0) # <--------
        lambda_eq = -b / 8.0 + math.sqrt((b**2) / 64.0 + b * blade_pitch_hover_rad / 6.0) # <--------

        # Tip speed = head speed (rpm) / 60 * 2pi * rotor diameter/2. Eq below is simplified.
        tip_speed_auto = self.var['_param_head_speed_set_point'] * np.pi * self.var['_param_diameter'] / 60.0

        # Calc the coefficient of thrust in the hover
        c_t_hover = 0.5 * b * (blade_pitch_hover_rad / 3.0 - lambda_eq / 2.0)
        self._lift_hover = c_t_hover * SSL_AIR_DENSITY * _disc_area * tip_speed_auto**2

        # Estimate rate of descent
        _est_rod = ((0.25 * self.var['_param_solidity'] * ASSUMED_CD0 / c_t_hover) + (c_t_hover**2 / (self.var['_param_solidity'] * ASSUMED_CD0))) * tip_speed_auto

        # Estimate rotor C_d
        self._c = (self._lift_hover / (_est_rod**2 * 0.5 * SSL_AIR_DENSITY * _disc_area)) * 1.15
        self._c = min(max(self._c, 0.7), 1.4)

        # Calc flare altitude
        des_spd_fwd = self.var['_param_target_speed'] * 0.01

        flare_alt, delta_t_flare = self.calc_flare_alt(-_est_rod, des_spd_fwd)
        return (flare_alt, self._lift_hover/GRAVITY_MSS, delta_t_flare)

    def calc_flare_alt(self, sink_rate, fwd_speed):

        # Compute speed module and glide path angle during descent
        speed_module = max(norm(sink_rate, fwd_speed), 0.1)
        glide_angle = safe_asin(np.pi / 2 - (fwd_speed / speed_module))

        # Estimate inflow velocity at beginning of flare
        inflow = - speed_module * math.sin(glide_angle + deg2rad(AP_ALPHA_TPP))

        # Estimate flare duration
        k_1 = np.abs((-sink_rate + 0.001 - math.sqrt(self._lift_hover / self._c))/(-sink_rate + 0.001 + math.sqrt(self._lift_hover / self._c)))
        k_2 = np.abs((inflow - math.sqrt(self._lift_hover / self._c)) / (inflow + math.sqrt(self._lift_hover / self._c)))
        delta_t_flare = (0.5 * (self._lift_hover / (GRAVITY_MSS * self._c)) * math.sqrt(self._c / self._lift_hover) * math.log(k_1)) - (0.5 * (self._lift_hover / (GRAVITY_MSS * self._c)) * math.sqrt(self._c / self._lift_hover) * math.log(k_2))

        # Estimate flare delta altitude
        sq_gravity = GRAVITY_MSS**2
        a = (2 * math.sqrt(self._c * sq_gravity / self._lift_hover )) * delta_t_flare + (2 * math.sqrt(self._c * sq_gravity / self._lift_hover )) * (0.5 * (self._lift_hover / (GRAVITY_MSS * self._c)) * math.sqrt(self._c / self._lift_hover) * math.log(k_1))
        x = 1 - math.exp(a)
        s = 1 - math.exp((2 * math.sqrt(self._c * sq_gravity / self._lift_hover )) * (0.5 * (self._lift_hover/(GRAVITY_MSS * self._c)) * math.sqrt(self._c / self._lift_hover) * math.log(k_1)))
        d = math.sqrt(self._lift_hover / self._c)
        flare_distance = ((2 * d / (2 * math.sqrt(self._c * sq_gravity / self._lift_hover ))) * (a - math.log(abs(x)) - (2 * math.sqrt(self._c * sq_gravity / self._lift_hover )) * (0.5 * (self._lift_hover / (GRAVITY_MSS * self._c)) * math.sqrt(self._c / self._lift_hover) * math.log(k_1)) + math.log(abs(s)))) - d * delta_t_flare
        delta_h = -flare_distance * math.cos(deg2rad(AP_ALPHA_TPP))

        # Estimate altitude to begin collective pull
        _cushion_alt = (-(sink_rate * math.cos(deg2rad(AP_ALPHA_TPP))) * self.var['_t_tch']) * 100.0

        # Total delta altitude to ground
        _flare_alt_calc = _cushion_alt + delta_h * 100.0

        return _flare_alt_calc*1e-2, delta_t_flare
# =====================================================================================


def plot_single_var_sensitivity(obj, var_name, var_array):
    flare_alt = []
    mass_est = []
    other_var = []
    for v in var_array:
        obj.var[var_name] = v
        alt, mass, delta_t_f = obj.initial_flare_estimate()
        flare_alt.append(alt)
        mass_est.append(mass)
        other_var.append(delta_t_f)

    # Plot against flare alt
    fig, ax = plt.subplots()
    ax.plot(var_array, flare_alt)
    ax.set_xlabel(var_name)
    ax.set_ylabel('Flare Alt (m)')

    # Plot against mass estimate
    fig, ax = plt.subplots()
    ax.plot(var_array, mass_est)
    ax.set_xlabel(var_name)
    ax.set_ylabel('Mass Est (kg)')

     # Plot against inflow ratio
    fig, ax = plt.subplots()
    ax.plot(var_array, other_var)
    ax.set_xlabel(var_name)
    ax.set_ylabel('Flare Time ()')

    obj.reset_to_defaults()



if __name__=='__main__':
    # Setup flare object to do calculations
    flare_obj = Flare_Calc_Pre_Changes()
    # flare_obj = Flare_Calc()

    N_PTS = 100000

    solidity = np.linspace(0.03, 0.10, N_PTS)
    c_l_alpha = np.linspace(np.pi, np.pi*2, N_PTS)
    diameter = np.linspace(0.72, 3.0, N_PTS)
    blade_pitch_hover_deg = np.linspace(0.1, 10.0, N_PTS)
    head_speed = np.linspace(600, 1500, N_PTS)
    target_speed = np.linspace(800, 1600, N_PTS)

    plot_single_var_sensitivity(flare_obj, '_param_solidity', solidity)
    plot_single_var_sensitivity(flare_obj, '_c_l_alpha', c_l_alpha)
    plot_single_var_sensitivity(flare_obj, '_param_diameter', diameter)
    plot_single_var_sensitivity(flare_obj, 'blade_pitch_hover_deg', blade_pitch_hover_deg)
    plot_single_var_sensitivity(flare_obj, '_param_head_speed_set_point', head_speed)
    plot_single_var_sensitivity(flare_obj, '_param_target_speed', target_speed)





    plt.show()
