from rome.frame import register_messages

register_messages(
  (0x20, [
    # __ ORDERS __
    # turn on control systems if activate is TRUE, turn it off otherwise
    ('asserv_activate', [('activate','uint8')]),

    # order robot to go directly to point (x,y) then turn to heading a 
    ('asserv_goto_xy',     [('x','dist'),('y','dist'),('a','angle')]),
    # same as asserv_goto_xy, but relative to previous coordinates
    ('asserv_goto_xy_rel', [('x','dist'),('y','dist'),('a','angle')]),
    # move robot straight ahead/backward depending on d sign 
    ('asserv_advance',     [('d','dist')]),
    # turn robot to heading a 
    ('asserv_turn',        [('a','angle')]),
    # turn robot heading by a 
    ('asserv_turn_rel',    [('a','angle')]),
    
    # autoset robot
    ('asserv_autoset', [('side','uint8'),('x','float'),('y','float')]),

    # __ SETTERS __
    # set robot current position (will force and reset position system to this position)
    ('asserv_set_xya', [('x','dist'),('y','dist'),('a','angle')]),
    ('asserv_set_xy',  [('x','dist'),('y','dist')]),
    ('asserv_set_a',   [('a','angle')]),
    # set control system PIDs
    ('asserv_set_x_pid', [('p','uint16'),('i','uint16'),('d','uint16')]),
    ('asserv_set_y_pid', [('p','uint16'),('i','uint16'),('d','uint16')]),
    ('asserv_set_a_pid', [('p','uint16'),('i','uint16'),('d','uint16')]),
    # set control system Qramps
    ('asserv_set_a_qramp', [('dot','uint16'),('dotdot','uint16')]),
    # set control system htajectory parameters
    ('asserv_set_htraj_xy_cruise',   [('speed','float'),('acc','float')]),
    ('asserv_set_htraj_xy_steering', [('speed','float'),('acc','float')]),
    ('asserv_set_htraj_xy_stop',     [('speed','float'),('acc','float')]),

    ('asserv_set_htraj_xysteering_window',  [('r','float')]),
    ('asserv_set_htraj_xystop_window',      [('r','float')]),
    ('asserv_set_htraj_astop_window',       [('r','float')]),

    # __ TELEMETRY __
    # XYA position
    ('asserv_tm_xya',   [('x','dist'), ('y','dist'), ('a','angle')]),
    # XYA PIDs telemetry
    ('asserv_tm_x_pid', [('input','int32'),('error','int32'),('out','int32')]),
    ('asserv_tm_y_pid', [('input','int32'),('error','int32'),('out','int32')]),
    ('asserv_tm_a_pid', [('input','int32'),('error','int32'),('out','int32')]),
    # htrajectory internal state
    ('asserv_tm_htraj_done', [('xy','uint8'),('a','uint8')]),
    ('asserv_tm_htraj_autoset_done', [('b','uint8')]),
    ('asserv_tm_htraj_state',      [('state','uint8')]),
    ('asserv_tm_htraj_carrot_xy', [('x','dist'),('y','dist')]),
    ('asserv_tm_htraj_speed',      [('speed','float')]),
    ('asserv_tm_htraj_path_index', [('i','uint8'),('size','uint8')]),
    # motors consigns
    ('asserv_tm_motors', [('m1','int32'),('m2','int32'),('m3','int32')]),

    # __ DEBUG __
    # all purposes debug message
    ('asserv_debug', [('v','uint16')]),
    ]),
)
