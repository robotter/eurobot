from rome.frame import register_messages

def _acked(name, params):
  params.insert(0, ('fid','uint8'))
  return (name, params)

register_messages(
  (0x10, [
    # __ MISC __
    ('ack', [('fid','uint8')]),
    ]),

  (0x20, [
    # __ ORDERS __
    # turn on control systems if activate is TRUE, turn it off otherwise
    _acked('asserv_activate', [('activate','uint8')]),

    # order robot to go directly to point (x,y) then turn to heading a 
    _acked('asserv_goto_xy',     [('x','dist'),('y','dist'),('a','angle')]),
    # same as asserv_goto_xy, but relative to previous coordinates
    _acked('asserv_goto_xy_rel', [('x','dist'),('y','dist'),('a','angle')]),
    
    # autoset robot
    _acked('asserv_autoset', [('side','uint8'),('x','float'),('y','float')]),

    # __ SETTERS __
    # set robot current position (will force and reset position system to this position)
    _acked('asserv_set_xya', [('x','dist'),('y','dist'),('a','angle')]),
    _acked('asserv_set_xy',  [('x','dist'),('y','dist')]),
    _acked('asserv_set_a',   [('a','angle')]),
    # set control system PIDs
    _acked('asserv_set_x_pid', [('p','uint16'),('i','uint16'),('d','uint16'),
                                  ('max_in','int32'),('max_I','int32'),('max_out','int32')]),
    _acked('asserv_set_y_pid', [('p','uint16'),('i','uint16'),('d','uint16'),
                                  ('max_in','int32'),('max_I','int32'),('max_out','int32')]),
    _acked('asserv_set_a_pid', [('p','uint16'),('i','uint16'),('d','uint16'),
                                  ('max_in','int32'),('max_I','int32'),('max_out','int32')]),
    # set control system Qramps
    _acked('asserv_set_a_qramp', [('dot','uint16'),('dotdot','uint16')]),
    # set control system htajectory parameters
    _acked('asserv_set_htraj_xy_cruise',   [('speed','float'),('acc','float')]),
    _acked('asserv_set_htraj_xy_steering', [('speed','float'),('acc','float')]),
    _acked('asserv_set_htraj_xy_stop',     [('speed','float'),('acc','float')]),

    _acked('asserv_set_htraj_xysteering_window',  [('r','float')]),
    _acked('asserv_set_htraj_stop_windows',      [('xy','float'), ('angle','float')]),

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

  (0x50, [
    # __ ORDERS __
    # set arm position
    _acked('meca_set_arm', [('upper','int32'),('elbow','int32'),('wrist','int32')]),
    # set sucker #n on / off (default off)
    _acked('meca_set_sucker', [('n','uint8'),('active','uint8')]),
    # set pump #n on / off (default off)
    _acked('meca_set_pump', [('n','uint8'),('active','uint8')]),
    # activate/deactivate motor power
    _acked('meca_set_power', [('active','uint8')]),
    # set servo #n
    _acked('meca_set_servo', [('n','uint8'), ('position','int16')]),

    # __ TELEMETRY __
    # return arm position
    ('meca_tm_arm', [('upper','int16'),('elbow','int16'),('wrist','int16')]),
    # return pressure sensors
    ('meca_tm_suckers', [('a','uint8'),('b','uint8')]),
   ]),
)
