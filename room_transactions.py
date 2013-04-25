#!/usr/bin/env python

from perlimpinpin.payload.room.transaction import *

transactions = register_groups(

    # Robot movement
    (0x10, [
      Command('asserv_get_position', [], [('x', 'dist'), ('y', 'dist'), ('a', 'angle')],
        desc="Get robot position"),
      Order('asserv_set_position', [('x', 'dist'), ('y', 'dist'), ('a', 'angle')],
        desc="Set robot position"),

      Order('asserv_goto_xy', [('x', 'dist'), ('y', 'dist')],
        desc="Move to given absolute position coordinates"),
      Order('asserv_goto_a', [('a', 'angle')],
        desc="Move to given absolute angle"),

      ]),

    # R3D2
    (0x30, [
      # detection
      Event('r3d2_detected', [('i', 'uint8'), ('a', 'uangle'), ('r', 'udist')],
        desc="Object detected at given polar coordinates"),
      Event('r3d2_disappeared', [('i', 'uint8')],
        desc="A previously detected object just disappeared"),

      # update state
      Order('r3d2_set_state', [('state', 'bool')],
        desc="Enable or disable R3D2 (motor and sensor)"),
      Order('r3d2_set_motor_speed', [('speed', 'uint16')],
        desc="Set R3D2 motor speed"),

      # configuration
      Command('r3d2_get_conf', [], [
        ('motor_speed', 'uint16'), ('motor_timeout', 'uint8'),
        ('angle_offset', 'angle'), ('dist_coef', 'uint16')],
        desc="Retrieve R3D2 configuration values"),
      Order('r3d2_set_conf', [
        ('motor_speed', 'uint16'), ('motor_timeout', 'uint8'),
        ('angle_offset', 'angle'), ('dist_coef', 'uint16')],
        desc="Set R3D2 configuration values"),
      Order('r3d2_load_conf', [],
        desc="Load R3D2 configuration from EEPROM"),
      Order('r3d2_save_conf', [],
        desc="Save R3D2 configuration to EEPROM"),

      # calibration
      Order('r3d2_calibrate_angle', [('a', 'uangle')],
        desc="Calibrate angle offset"),
      Order('r3d2_calibrate_dist', [('r', 'udist')],
        desc="Calibrate distance coefficient"),

      ]),

    # PMI
    (0xA0, [
      # position configuration
      Order('pmi_position_set_conf', [
        ('left_wheel_ratio', 'float'),
        ('right_wheel_ratio', 'float'),
        ('tick_p_mm', 'float'),
        ('tick_p_180deg', 'float'),
        ], desc="Set position manager configuration"),
      Order('pmi_position_save_conf', [],
        desc="Save position manager configuration to EEPROM"),
      # ramp configurations
      Order('pmi_ramp_dist_set_conf', [('a_max', 'float'), ('v_max', 'float')],
        desc="Set distance ramp configuration"),
      Order('pmi_ramp_dist_save_conf', [],
        desc="Save distance ramp configuration to EEPROM"),
      Order('pmi_ramp_angle_set_conf', [('a_max', 'float'), ('v_max', 'float')],
        desc="Set distance ramp configuration"),
      Order('pmi_ramp_angle_save_conf', [],
        desc="Save angle ramp configuration to EEPROM"),
      # pid configurations
      Order('pmi_pid_dist_set_conf', [
        ('kd', 'float'),
        ('ki', 'float'),
        ('kp', 'float'),
        ('d_alpha', 'float'),
        ('max_integral', 'float'),
        ('max_output', 'float'),
        ], desc="Set distance PID configuration"),
      Order('pmi_pid_dist_save_conf', [],
        desc="Save distance PID configuration to EEPROM"),
      Order('pmi_pid_angle_set_conf', [
        ('kd', 'float'),
        ('ki', 'float'),
        ('kp', 'float'),
        ('d_alpha', 'float'),
        ('max_integral', 'float'),
        ('max_output', 'float'),
        ], desc="Set distance PID configuration"),
      Order('pmi_pid_angle_save_conf', [],
        desc="Save angle PID configuration to EEPROM"),
      # all
      Order('pmi_save_all_conf', [],
        desc="Save all configurations to EEPROM"),

      ]),

    )

