import numpy as np
from matplotlib import pyplot as plt
import math

def linear_interpolate(
    output_low: float,
    output_high: float,
    input_value: float,
    input_low: float,
    input_high: float
) -> float:
    """
    Linearly interpolate to map input_value from the range [input_low, input_high]
    to the range [output_low, output_high]. If input_low > input_high, the ranges
    are swapped to support either polarity.

    :param output_low:  Lower bound of the output range.
    :param output_high: Upper bound of the output range.
    :param input_value: The value to map.
    :param input_low:   Lower bound of the input range.
    :param input_high:  Upper bound of the input range.
    :return:            The interpolated value.
    """
    # support either polarity of the input range
    if input_low > input_high:
        input_low, input_high = input_high, input_low
        output_low, output_high = output_high, output_low

    # clamp to the output range at the ends
    if input_value <= input_low:
        return output_low
    if input_value >= input_high:
        return output_high

    # compute interpolation factor
    p = (input_value - input_low) / (input_high - input_low)
    return output_low + p * (output_high - output_low)


def velocity_target(initial_climb_rate: float,
                    hagl_entry: float,
                    final_climb_rate: float,
                    hagl_exit: float,
                    current_hagl: float,
                    k: float = 1.0) -> float:
    """
    Exponentially interpolate the climb/descent rate from initial_climb_rate
    at hagl_entry down to final_climb_rate at hagl_exit, based on current_hagl.

    Parameters
    ----------
    initial_climb_rate : float
        Rate (m/s) at the start height (hagl_entry).
    hagl_entry : float
        Height above ground (m) where interpolation starts.
    final_climb_rate : float
        Rate (m/s) at the end height (hagl_exit).
    hagl_exit : float
        Height above ground (m) where interpolation ends.
    current_hagl : float
        Current height above ground (m).
    k : float, optional
        Exponential curve constant.  k=0 → linear; k>0 → exponential shape 
        (default 1.0).

    Returns
    -------
    float
        Target rate (m/s) at current_hagl.
    """
    # support either ordering of entry/exit
    if hagl_entry < hagl_exit:
        hagl_entry, hagl_exit = hagl_exit, hagl_entry
        initial_climb_rate, final_climb_rate = final_climb_rate, initial_climb_rate

    # avoid zero‐division
    span = hagl_entry - hagl_exit
    if span <= 0:
        return final_climb_rate

    # clamp current height into [hagl_exit, hagl_entry]
    h = max(min(current_hagl, hagl_entry), hagl_exit)

    # normalized position in [0,1], 1=at entry, 0=at exit
    t = (h - hagl_exit) / span

    # interpolation factor
    if k == 0:
        ratio = t
    else:
        # math.expm1(x) = exp(x) - 1, more accurate for small x
        denom = math.expm1(k)
        ratio = math.expm1(k * t) / denom

    # blend between final and initial
    return final_climb_rate + (initial_climb_rate - final_climb_rate) * ratio


def expo_curve(alpha, x):
    return (1.0 - alpha) * x + alpha * x * x * x

def thst_expo_curve(expo, x):
    return ((expo - 1.0) + np.sqrt((1.0 - expo) * (1.0 - expo) + 4.0 * expo * x)) / (2.0 * expo)


def expm1_interpolate(
    output_low: float,
    output_high: float,
    input_value: float,
    input_low: float,
    input_high: float,
    k: float
) -> float:
    """
    Interpolate a value along an exponential curve with exponent k.

    :param output_low:    Value at input_low
    :param output_high:   Value at input_high
    :param input_value:   Current input
    :param input_low:     Lower bound of input range
    :param input_high:    Upper bound of input range
    :param k:             Exponential exponent (k=0 yields linear)
    :return:              Interpolated output
    """
    # Ensure input_low <= input_high; swap if needed
    if input_low > input_high:
        input_low, input_high = input_high, input_low
        output_low, output_high = output_high, output_low

    # Compute span and handle zero case
    span = input_high - input_low
    if span == 0.0:
        return output_high

    # Clamp input_value into [input_low, input_high]
    if input_value <= input_low:
        return output_low
    if input_value >= input_high:
        return output_high

    # Normalized input to be between 0 and 1
    x = (input_value - input_low) / span

    # Compute exponential interpolation factor
    if k == 0.0:
        y = x
    else:
        # math.expm1 provides exp(k)-1 with good precision for small k
        denom = math.expm1(k)
        y = math.expm1(k * x) / denom

    # Blend between output_low and output_high
    return output_low + (output_high - output_low) * y


## Plot the speed height diagram for hover autorotation case
def speed_hgt():
    cruise_speed = 12
    flare_min_hgt = 10
    touchdown_min_hgt = 2

    speeds = np.linspace(0, cruise_speed, 100)

    heights = []

    for s in speeds:
        hgt = linear_interpolate(flare_min_hgt, touchdown_min_hgt, s, 0.0, cruise_speed)
        heights.append(hgt)

    fig, ax = plt.subplots()
    ax.plot(speeds, heights)
    ax.set_xlabel('Speed (m/s)')
    ax.set_ylabel('Height (AGL) (m)')



## Plot the climb rate target trajectory for touch down
def touchdown_trajectory():

    fig, ax = plt.subplots()
    ax.set_ylabel('Height AGL (m)')
    ax.set_xlabel('Vz (m/s)')

    # exp = [-1.0, -0.5, 0.0, 0.5, 1.0]
    # exp = [-2.0, -1.0, 0.0, 1.0, 2.0]
    exp = [-10.0, -5.0, -2.5, 0.0, 2.5, 5.0, 10.0]
    for e in exp:

        heights, vz = run_td_case(e)


        ax.plot(vz, heights, label='Exp = %.1f' % e)

    plt.legend()


def run_td_case(exponent):
    init_hagl = 5 #(m)
    exit_hagl = 0 #(m)

    init_climb_rate = -2.0 #(m/s)
    exit_climb_rate = 0.0 #(m/s)

    heights = np.linspace(init_hagl, exit_hagl, 100)

    vz = []
    for h in heights:
        # v = velocity_target(init_climb_rate,
        #                 init_hagl,
        #                 exit_climb_rate,
        #                 exit_hagl,
        #                 h,
        #                 exponent)
        
        # Normalise the height measurement
        # pos = h / (init_hagl - exit_hagl)
        # pos = min(max(pos, exit_hagl), init_hagl)

        # # Calc the normalised velocity target
        # # v_norm = expo_curve(exponent, pos)
        # v_norm = thst_expo_curve(exponent, pos)

        # # Calc the velocity target
        # v = (init_climb_rate - exit_climb_rate) * v_norm + exit_climb_rate

        v = expm1_interpolate(exit_climb_rate, init_climb_rate, h, exit_hagl, init_hagl, exponent)

        vz.append(v)

    return heights, vz














if __name__ == "__main__":
    # speed_hgt()
    touchdown_trajectory()

    plt.show()