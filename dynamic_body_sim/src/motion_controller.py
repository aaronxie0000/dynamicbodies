

def compute_torque(desired_pos, cur_poss, cur_vels):
    cur_pos = cur_poss[0]
    cur_vel = cur_vels[0]

    desired_vel = 0.0

    kp = 10
    kd = 1

    return kp * (desired_pos - cur_pos) + kd * (desired_vel - cur_vel)


