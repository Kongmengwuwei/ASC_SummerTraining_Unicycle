"""Mock parity with the MCU explicit runtime control allowlist."""
LIVE_CONTROL_PARAMETERS = frozenset(['run_speed_straight', 'run_speed_curve', 'run_speed_cross', 'run_speed_ring', 'run_speed_ramp', 'run_speed_lost', 'run_accel_mps2', 'run_decel_mps2', 'direction_pixel_kp', 'direction_heading_kp', 'direction_curve_kff', 'direction_rate_kd', 'lean_roll_kp', 'lean_max_angle'])
