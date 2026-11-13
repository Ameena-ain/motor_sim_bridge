import math


def cmd_vel_to_rpm(v, w, r, L):
    v_left = v - w * L / 2
    v_right = v + w * L / 2
    left_rpm = v_left / r * 60 / (2 * math.pi)
    right_rpm = v_right / r * 60 / (2 * math.pi)
    return left_rpm, right_rpm


def motor_step(current_rpm, target_rpm, dt, tau):
    gap = target_rpm - current_rpm
    new_rpm = current_rpm + gap * (dt / tau)
    return new_rpm
