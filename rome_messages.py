from rome.frame import register_messages, Order, rome_types

register_messages(
    (0x10, [
      (Order, 'start_timer', []),
      (Order, 'reset', []),
      ]),

    # ASSERV
    (0x20, [
      # __ ORDERS __
      # turn on control systems if activate is TRUE, turn it off otherwise
      (Order, 'asserv_activate', [('activate','bool')]),

      # order robot to go directly to point (x,y) then turn to heading a
      (Order, 'asserv_goto_xy', [('x','dist'), ('y','dist'), ('a','angle')]),
      # same as asserv_goto_xy, but relative to previous coordinates
      (Order, 'asserv_goto_xy_rel', [('x','dist'), ('y','dist'), ('a','angle')]),
      
      # order robot to go directly to point (x,y) then turn to heading a
      (Order, 'asserv_goto_xya_panning', [('x','dist'), ('y','dist'), ('pan_angle', 'angle')]),
      (Order, 'asserv_goto_xya_synced', [('x','dist'), ('y','dist'), ('a', 'angle')]),

      # order to run along a trajectory (xy array size must be even)
      (Order, 'asserv_run_traj', [('a','angle'),('xy','dist[]')]),

      # autoset robot
      #   robot side: 0: left, 1: right, 2: back
      #   table side: 0: none, 1: left, 2: right, 3: up, 4: down
      (Order, 'asserv_autoset', [('robot_side','uint8'), ('table_side','uint8'), ('x','float'), ('y','float')]),

      # __ SETTERS __
      # set robot current position (will force and reset position system to this position)
      (Order, 'asserv_set_xya', [('x','dist'), ('y','dist'), ('a','angle')]),
      (Order, 'asserv_set_xy', [('x','dist'), ('y','dist')]),
      (Order, 'asserv_set_a', [('a','angle')]),
      # set control system PIDs
      (Order, 'asserv_set_x_pid', [('p','uint16'), ('i','uint16'), ('d','uint16'),
        ('max_in','int32'), ('max_I','int32'), ('max_out','int32')]),
      (Order, 'asserv_set_y_pid', [('p','uint16'), ('i','uint16'), ('d','uint16'),
        ('max_in','int32'), ('max_I','int32'), ('max_out','int32')]),
      (Order, 'asserv_set_a_pid', [('p','uint16'), ('i','uint16'), ('d','uint16'),
        ('max_in','int32'), ('max_I','int32'), ('max_out','int32')]),
      # set control system Qramps
      (Order, 'asserv_set_a_qramp', [('dot','uint16'), ('dotdot','uint16')]),
      # set control system htajectory parameters
      (Order, 'asserv_set_htraj_a_speed', [('speed','float'), ('acc','float')]),
      (Order, 'asserv_set_htraj_xy_cruise', [('speed','float'), ('acc','float')]),
      (Order, 'asserv_set_htraj_xy_steering', [('speed','float'), ('acc','float')]),
      (Order, 'asserv_set_htraj_xy_stop', [('speed','float'), ('acc','float')]),

      (Order, 'asserv_set_htraj_xysteering_window', [('r','float')]),
      (Order, 'asserv_set_htraj_stop_windows', [('xy','float'), ('angle','float')]),

      (Order, 'asserv_set_zgyro_scale', [('zscale','float')]),

      (Order, 'asserv_set_servo', [('id','uint8'),('value','uint16')]),

      (Order, 'asserv_set_cmatrix', [('m%d'%i,'float')for i in xrange(0,9)]),

      # __ TELEMETRY __
      # Encoder raw data
      ('asserv_tm_encoder_raw', [('enc0','int16'), ('enc1','int16'), ('enc2','int16')]),
      # XYA position
      ('asserv_tm_xya', [('x','dist'), ('y','dist'), ('a','angle')]),
      # XYA PIDs telemetry
      ('asserv_tm_x_pid', [('input','int32'), ('error','int32'), ('out','int32')]),
      ('asserv_tm_y_pid', [('input','int32'), ('error','int32'), ('out','int32')]),
      ('asserv_tm_a_pid', [('input','int32'), ('error','int32'), ('out','int32')]),
      # htrajectory internal state
      ('asserv_tm_htraj_done', [('xy','bool'), ('a','bool')]),
      ('asserv_tm_htraj_autoset_done', [('b','bool')]),
      ('asserv_tm_htraj_state', [('state','uint8')]),
      ('asserv_tm_htraj_carrot_xy', [('x','dist'), ('y','dist')]),
      ('asserv_tm_htraj_speed', [('speed','float')]),
      ('asserv_tm_htraj_path_index', [('i','uint8'), ('size','uint8')]),
      # motors consigns
      ('asserv_tm_motors', [('m1','int32'), ('m2','int32'), ('m3','int32')]),
      # gp2 raw values
      ('asserv_tm_gp2_raws', [('g0','uint16'), ('g1','uint16'), ('g2','uint16')]),
      # gp2 detection vector
      ('asserv_tm_gp2_det', [('g0','uint16'), ('g1','uint16'), ('g2','uint16')]),
      # match timer
      ('asserv_tm_match_timer', [('seconds','int16')]),
      # gyro calibration status
      ('asserv_tm_gyro_calibration', [('active','bool')]),

      # __ DEBUG __
      # all purposes debug message
      ('asserv_debug', [('v','uint16')]),

      # turn on/off gyro calibration (warning : asserv must be deactivated)
      (Order, 'asserv_calibrate', [('b','uint8')]),

      # turn on/off avoidance
      (Order, 'asserv_avoidance', [('b','bool')]),
        
      # turn on/off gyro integration
      (Order, 'asserv_gyro_integration', [('b','bool')]),

      # bumper state (used by galipeur autoset)
      ('asserv_tm_bumpers_state', [('b0','bool'),('b1','bool')]),

      ]),

    # STRATEGY
    (0x60, [
      # __ TELEMETRY __
      # robot main battery voltage (in mv)
      ('strat_tm_battery', [('voltage','uint16')]),
      ]),

    # R3D2
    (0x70, [
      ('r3d2_tm_detection', [('i','uint8'), ('detected','bool'), ('a','angle'), ('r','dist')]),
      ('r3d2_tm_arcs', [('i','uint8'), ('a1','angle'), ('a2','angle')]),

      (Order,'r3d2_set_rotation', [('speed_rpm','uint16'),('threshold_percent','uint8')]),
      (Order,'r3d2_set_blind_spot', [('begin','angle'),('end','angle')])
      ]),

    # color_sensor
    (0x80, [
      # Detection of object i (i is 0 or 1)
      ('color_sensor_tm_detection', [('detected','bool'), ('color', 'uint8')]),
      ('color_sensor_tm_dist_debug', [('distance','uint16'), ('detected','bool'), ('color', 'uint8')]),
      (Order, 'color_sensor_set_color_filter', [('i', 'uint8'), ('red_threshold','uint16'), ('green_threshold','uint16'), ('blue_threshold','uint16')]),
      (Order, 'color_sensor_set_dist_threshold', [('low_threshold','uint16'), ('high_threshold','uint16'), ('consecutive_detect_threshold','uint8')]),
      ]),

    # MECA
    (0xA0, [
      # __ ORDERS __
      # activate/deactivate motor power
      (Order, 'meca_set_power', [('active','bool')]),
      # set servo #n
      (Order, 'meca_set_arm', [('position','int16')]),
      # meca spot elevator commands
      rome_types.rome_enum('meca_command', (
          'none',
          'prepare_pick_spot',
          'pick_spot',
          'discharge_spot_stack',
          'release_spot_stack',
          'pick_bulb',
          'prepare_bulb',
          'prepare_cup',
          'unload_cup',
          'pick_cup',
          'reset_elevator',
          )),
      (Order, 'meca_cmd', [('cmd','meca_command'),('side','int8')]),
      # carpets
      (Order, 'meca_carpet_lock', [('n','uint8')]),
      (Order, 'meca_carpet_unlock', [('n','uint8')]),
      # sand roller
      (Order, 'meca_set_sand_roller', [('active','bool')]),
      # generic analog servo commands
      (Order, 'meca_set_servo', [('id','uint8'),('value','uint16')]),
      # __ TELEMETRY __
      # match timer
      ('meca_tm_match_timer', [('seconds','int16')]),
      # spot elevators
      rome_types.rome_enum('meca_elevator_state', (
          'busy',  # elevator won't accept commands, robot shouldn't move
          'ground_clear',  # elevator won't accept commands, robot can move
          'ready',  # elevator ready for new commands
          )),
      ('meca_tm_left_elevator'  ,[('state','meca_elevator_state'),('nb_spots','int8')]),
      ('meca_tm_right_elevator' ,[('state','meca_elevator_state'),('nb_spots','int8')]),
      ]),

  )

