#!/usr/bin/env python

from perlimpinpin.payload.room.transaction import *

transactions = register_groups(

    # General
    (0x00, [
      # 1 for red, 2 for blue
      Event('robot_color', [('color', 'uint8')],
        desc="Announce robot color"),
      ]),

    # Robot movement
    (0x10, [
      Command('asserv_get_position', [], [('x', 'dist'), ('y', 'dist'), ('a', 'angle')],
        desc="Get robot position"),
      Order('asserv_set_position', [('x', 'dist'), ('y', 'dist'), ('a', 'angle')],
        desc="Set robot position"),

      Order('asserv_goto_xy', [('x', 'dist'), ('y', 'dist')],
        desc="Move to given absolute position coordinates"),
      Order('asserv_goto_d', [('d', 'dist')],
        desc="Move ahead"),
      Order('asserv_goto_a', [('a', 'angle')],
        desc="Move to given absolute angle"),
      
      Order('asserv_activate', [('activate', 'bool')],
        desc="Activate/deactivate control system"),

      Command('asserv_status', [], [('arrived_a', 'bool'), ('arrived_xy', 'bool')]),
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

    # Meca
    (0x50, [
      Order('meca_balloon_tap', [('open', 'bool')],
        desc="Open/close the balloon tap"),
      Order('meca_set_cake_angle', [('a', 'angle')], 
        desc="Set angle position along the cake"),
      Command('meca_get_cake_angle', [], [('a', 'angle')],
        desc="Get angle position along the cake"),

      Order('meca_set_arm_mode', [('mode', 'uint8')],
        desc="Set arm mode for caking"),

      ]),

    # AX-12
    (0x80, [
      # generic read/write
      Command('ax12_read_byte', [('id', 'uint8'), ('addr', 'uint8')], [('data', 'uint8'), ('error', 'uint8')],
        desc="Read a byte from an AX-12 memory"),
      Command('ax12_read_word', [('id', 'uint8'), ('addr', 'uint8')], [('data', 'uint16'), ('error', 'uint8')],
        desc="Read a word (2 bytes) from an AX-12 memory"),
      Command('ax12_write_byte', [('id', 'uint8'), ('addr', 'uint8'), ('data', 'uint8')], [('error', 'uint8')],
        desc="Write a byte to an AX-12 memory"),
      Command('ax12_write_word', [('id', 'uint8'), ('addr', 'uint8'), ('data', 'uint16')], [('error', 'uint8')],
        desc="Write a word (2 bytes) to an AX-12 memory"),

      # helpers to get/set several common values at once
      Order('ax12_move', [('id', 'uint8'), ('pos', 'uint16'), ('speed', 'uint16')],
        desc="Move AX-12 to a given position at a given speed"),
      Command('ax12_state', [('id', 'uint8')], [('pos', 'uint16'), ('moving', 'bool')],
        desc="Get AX-12 current position and if it is moving"),

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

      Order('pmi_go', [],
        desc="Tell the PMI to start the match"),
      Event('pmi_battery_voltage', [('decivolts','uint8')],
        desc="Broadcast battery voltage"),
    ]),
    # Galipeur
    (0xC0, [
      # position configuration
      Order('galipeur_force_thrust', [('vx', 'int32'), ('vy', 'int32'), ('omegaz', 'int32')],
        desc="Apply thrust to motors, asserv MUST BE deactivated"),
      ]),
    
    )

