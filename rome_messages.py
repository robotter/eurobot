from rome.frame import register_messages, Order, rome_types

rome_types.rome_enum('device', {
  'galipeur_strat': 10,     #xbee addr 0x20
  'galipeur_asserv': 11,
  'galipeur_meca': 12,
  'galipeur_r3d2': 13,
  'galipette_strat': 20,    #xbee addr 0x20
  'galipette_asserv': 21,
  'galipette_meca': 22,
  'galipette_r3d2': 23,
  'boomotter': 30,          #xbee addr 0x10
  'pannotter': 40,          #xbee addr 0xAA
})

register_messages(
    (0x10, [
      (Order, 'start_timer', []),
      (Order, 'reset', []),
      ]),

    # ASSERV
    (0x20, [
      # __ ORDERS __
      # turn on control systems if activate is TRUE, turn it off otherwise
      (Order, 'asserv_activate', ['bool activate']),

      # order robot to go directly to point (x,y) then turn to heading a
      (Order, 'asserv_goto_xy', ['dist x', 'dist y', 'angle a']),
      # same as asserv_goto_xy, but relative to previous coordinates
      (Order, 'asserv_goto_xy_rel', ['dist x', 'dist y', 'angle a']),
      # order robot to go directly to point (x,y) while panning
      (Order, 'asserv_goto_xya_panning', ['dist x', 'dist y', 'angle pan_angle']),
      # order robot to go directly to point (x,y) and turng to heading a, simultaneously
      (Order, 'asserv_goto_xya_synced', ['dist x', 'dist y', 'angle a']),

      # order to run along a trajectory (xy array size must be even)
      (Order, 'asserv_run_traj', ['angle a', 'dist[] xy']),

      # autoset robot
      #   robot side: 0: left, 1: right, 2: back
      #   table side: 0: none, 1: left, 2: right, 3: up, 4: down
      (Order, 'asserv_autoset', ['uint8 robot_side', 'uint8 table_side', 'float x', 'float y']),

      # __ SETTERS __
      # set robot current position (will force and reset position system to this position)
      (Order, 'asserv_set_xya', ['dist x', 'dist y', 'angle a']),
      (Order, 'asserv_set_xy', ['dist x', 'dist y']),
      (Order, 'asserv_set_a', ['angle a']),
      # set control system PIDs
      (Order, 'asserv_set_x_pid', ['uint16 p', 'uint16 i', 'uint16 d',
        'int32 max_in', 'int32 max_I', 'int32 max_out']),
      (Order, 'asserv_set_y_pid', ['uint16 p', 'uint16 i', 'uint16 d',
        'int32 max_in', 'int32 max_I', 'int32 max_out']),
      (Order, 'asserv_set_a_pid', ['uint16 p', 'uint16 i', 'uint16 d',
        'int32 max_in', 'int32 max_I', 'int32 max_out']),
      # set control system Qramps
      (Order, 'asserv_set_a_qramp', ['uint16 dot', 'uint16 dotdot']),
      # set control system htajectory parameters
      (Order, 'asserv_set_htraj_a_speed', ['float speed', 'float acc']),
      (Order, 'asserv_set_htraj_xy_cruise', ['float speed', 'float acc']),
      (Order, 'asserv_set_htraj_xy_steering', ['float speed', 'float acc']),
      (Order, 'asserv_set_htraj_xy_stop', ['float speed', 'float acc']),

      (Order, 'asserv_set_htraj_xysteering_window', ['float r']),
      (Order, 'asserv_set_htraj_stop_windows', ['float xy', 'float angle']),

      (Order, 'asserv_set_zgyro_scale', ['float zscale']),

      (Order, 'asserv_set_servo', ['uint8 id', 'uint16 value']),

      (Order, 'asserv_set_cmatrix', ['float[9] m']),

      # __ TELEMETRY __
      # Encoder raw data
      ('asserv_tm_encoder_raw', ['int16 enc0', 'int16 enc1', 'int16 enc2']),
      # XYA position
      ('asserv_tm_xya', ['device device', 'dist x', 'dist y', 'angle a']),
      # XYA PIDs telemetry
      ('asserv_tm_x_pid', ['int32 input', 'int32 error', 'int32 out']),
      ('asserv_tm_y_pid', ['int32 input', 'int32 error', 'int32 out']),
      ('asserv_tm_a_pid', ['int32 input', 'int32 error', 'int32 out']),
      # htrajectory internal state
      ('asserv_tm_htraj_done', ['bool xy', 'bool a']),
      ('asserv_tm_htraj_autoset_done', ['bool b']),
      ('asserv_tm_htraj_state', ['uint8 state']),
      ('asserv_tm_htraj_carrot_xy', ['dist x', 'dist y']),
      ('asserv_tm_htraj_speed', ['float speed']),
      ('asserv_tm_htraj_path_index', ['uint8 i', 'uint8 size']),
      # motors consigns
      ('asserv_tm_motors', ['int32 m1', 'int32 m2', 'int32 m3']),
      # gp2 raw values
      ('asserv_tm_gp2_raws', ['uint16 g0', 'uint16 g1', 'uint16 g2']),
      # gp2 detection vector
      ('asserv_tm_gp2_det', ['uint16 g0', 'uint16 g1', 'uint16 g2']),
      # gyro calibration status
      ('asserv_tm_gyro_calibration', ['bool active']),

      # __ DEBUG __
      # all purposes debug message
      ('asserv_debug', ['uint16 v']),

      # turn on/off gyro calibration (warning : asserv must be deactivated)
      (Order, 'asserv_calibrate', ['uint8 b']),

      # turn on/off avoidance
      (Order, 'asserv_avoidance', ['bool b']),

      # turn on/off gyro integration
      (Order, 'asserv_gyro_integration', ['bool b']),

      # bumper state (used by galipette autoset)
      ('asserv_tm_bumpers_state', ['bool left', 'bool right']),

      ]),

    # COMMON
    (0x60, [
      # robot main battery voltage (in mv)
      ('tm_battery', ['device device', 'uint16 voltage']),
      ('tm_score',  ['device device', 'uint16 points']),
      ('tm_match_timer', ['device device', 'int16 seconds']),
      ('tm_robot_position', ['device device', 'int16 x', 'int16 y', 'angle a']),
    ]),

    # R3D2
    (0x70, [
      ('r3d2_tm_detection', ['uint8 i', 'bool detected', 'angle a', 'dist r']),
      ('r3d2_tm_arcs', ['uint8 i', 'angle a1', 'angle a2']),

      (Order,'r3d2_set_rotation', ['uint16 speed_rpm', 'uint8 threshold_percent']),
      (Order,'r3d2_set_blind_spot', ['angle begin', 'angle end'])
      ]),

    # color_sensor
    (0x80, [
      # Detection of object i (i is 0 or 1)
      ('color_sensor_tm_detection', ['bool detected', 'uint8 color']),
      ('color_sensor_tm_dist_debug', ['uint16 distance', 'bool detected', 'uint8 color']),
      (Order, 'color_sensor_set_color_filter', ['uint8 i', 'uint16 red_threshold', 'uint16 green_threshold', 'uint16 blue_threshold']),
      (Order, 'color_sensor_set_dist_threshold', ['uint16 low_threshold', 'uint16 high_threshold', 'uint8 consecutive_detect_threshold']),
      ]),

    # JeVois camera
    (0x90, [
      rome_types.rome_enum('jevois_color', (
        'none',
        'blue',
        'gold',
        )),
      ('jevois_tm_objects', [
        'jevois_color object_color',
        'dist x',
        'dist y',
        'dist z',
      ]),
    ]),

    # MECA
    (0xA0, [
      # __ ORDERS __
      # activate/deactivate motor power
      (Order, 'meca_set_power', ['bool active']),
      # set robot color
      (Order, 'meca_set_robot_color', ['bool yellow']),
      # meca commands
      (Order, 'meca_move_elevator', ['bool side', 'uint16 pos_mm']),
      (Order, 'meca_shutdown_elevator', ['bool side']),
      (Order, 'meca_take_cans', ['bool side']),
      (Order, 'meca_release_cans', ['bool side']),
      (Order, 'meca_deploy_wings', ['bool side']),
      (Order, 'meca_fold_wings'  , ['bool side']),
      (Order, 'meca_grab_wings'  , ['bool side']),

      # generic analog servo commands
      (Order, 'meca_set_servo', ['uint8 id', 'uint16 value']),

      # __ TELEMETRY __
      # meca state
      rome_types.rome_enum('meca_state', (
          'busy',  # meca won't accept commands and robot shouldn't move
          'ground_clear',  # meca won't accept commands but robot can move
          'ready',  # meca ready for new commands
          )),
      ('meca_tm_state', ['meca_state state']),
      ('meca_tm_arms_state', [
          'int16 l_pos',  # millimeters, -1 if unknown
          'int16 r_pos',  # millimeters, -1 if unknown
      ]),
    ]),

    # PATHFINDING
    (0xB0, [
      ('pathfinding_node', ['int8 i', 'int16 x', 'int16 y', 'int8[] neighbors']),
      ('pathfinding_path', ['int8[] nodes']),
    ]),

    # BOOMOTTER
    (0xB8, [
      ('boomotter_tm_battery', ['uint16 voltage']),
      (Order, 'boomotter_mp3_cmd', ['uint8 cmd', 'uint16 param']),
      rome_types.rome_enum('boomotter_mode', (
          'match',
          'music',
          'silent',
          'nyancat',
          'loituma',
      )),
      (Order, 'boomotter_set_mode', ['boomotter_mode mode']),
      (Order, 'boomotter_set_message', ['string msg']),
    ]),

    # PANNOTTER
    (0xBD, [
      (Order, 'pannotter_start_experience', []),
    ]),

    # SABOTTER
    (0xD0, [
      (Order, 'strat_set_servo', ['uint8 id', 'uint16 value']),
      (Order, 'strat_test', ['bool active']),
    ])
  )

