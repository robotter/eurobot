from rome.frame import register_messages, Order

register_messages(
    (0x10, [
      (Order, 'start_timer', []),
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

      # autoset robot, sides are (from 0 to 4): none, left, right, up, down
      (Order, 'asserv_autoset', [('side','uint8'), ('x','float'), ('y','float')]),

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
      (Order, 'asserv_set_htraj_xy_cruise', [('speed','float'), ('acc','float')]),
      (Order, 'asserv_set_htraj_xy_steering', [('speed','float'), ('acc','float')]),
      (Order, 'asserv_set_htraj_xy_stop', [('speed','float'), ('acc','float')]),

      (Order, 'asserv_set_htraj_xysteering_window', [('r','float')]),
      (Order, 'asserv_set_htraj_stop_windows', [('xy','float'), ('angle','float')]),

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

      # __ DEBUG __
      # all purposes debug message
      ('asserv_debug', [('v','uint16')]),

      # turn on/off gyro calibration (warning : asserv must be deactivated)
      (Order, 'asserv_calibrate', [('b','uint8')]),

      # turn on/off avoidance
      (Order, 'asserv_avoidance', [('b','bool')]),

      ]),

    # MECA
    (0x50, [
      # __ ORDERS __
      # set arm position
      (Order, 'meca_set_arm', [('upper','int16'), ('elbow','int16'), ('wrist','int16')]),
      # set sucker #n on / off (default off)
      (Order, 'meca_set_sucker', [('n','uint8'), ('active','bool')]),
      # set pump #n on / off (default off)
      (Order, 'meca_set_pump', [('n','uint8'), ('active','bool')]),
      # activate/deactivate motor power
      (Order, 'meca_set_power', [('active','bool')]),
      # set servo #n
      (Order, 'meca_set_servo', [('n','uint8'), ('position','int16')]),

      # __ TELEMETRY __
      # return arm position
      ('meca_tm_arm', [('upper','int16'), ('elbow','int16'), ('wrist','int16')]),
      # return pressure sensors
      ('meca_tm_suckers', [('a','bool'), ('b','bool')]),
      # match timer
      ('meca_tm_match_timer', [('seconds','int16')]),
      ]),

    # STRATEGY
    (0x60, [
      # __ TELEMETRY __
      # robot main battery voltage (in mv)
      ('strat_tm_battery', [('voltage','uint16')]),
      ]),

    # KATIOUCHA
    (0x65, [
      # fire n KATIOUCHA tubes
      (Order, 'katioucha_fire', [('n','uint8')]),
      #
      ('katioucha_tm_rounds_fired', [('i','uint8')]),
      ]),

    # R3D2
    (0x70, [
      # Detection of object i (i is 0 or 1)
      ('r3d2_tm_detection', [('i','int8'), ('detected','bool'), ('a','angle'), ('r','dist')]),
      (Order, 'r3d2_calibrate_angle', [('a','angle')]),
      (Order, 'r3d2_calibrate_dist', [('d','dist')]),
      (Order, 'r3d2_conf_load', []),
      (Order, 'r3d2_conf_save', []),
      (Order, 'r3d2_set_motor_speed', [('speed','uint16')]),
      ]),

  )

