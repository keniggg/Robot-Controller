def verify_force_hold(total_force_mn, hold_force_min_mn=800.0):
    return float(total_force_mn) >= float(hold_force_min_mn)
