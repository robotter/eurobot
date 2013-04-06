#!/usr/bin/env python

from perlimpinpin.payload.room.transaction import *

transactions = register_groups(

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

    )

