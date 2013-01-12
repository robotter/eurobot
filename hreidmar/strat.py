#!/usr/bin/env python2.7
import sys
import os
import traceback
import time
import threading
import signal
# expand PYTHONPATH to be able to find everything
_this_dir = os.path.abspath(os.path.dirname(__file__))
sys.path.append(_this_dir)
sys.path.append(os.path.dirname(_this_dir))

from pppcommon import *
from serverhub import HreidmarHub, hreidmar_nodes

COLOR_RED = 1
COLOR_BLUE = 2
color2name = {
    COLOR_RED: 'red',
    COLOR_BLUE: 'blue',
    }

pi = math.pi

board_addrs = [ x[0] for x in hreidmar_nodes ]


class StratHub(HreidmarHub, RoomHubMixin):

  def __init__(self, port=None):
    HreidmarHub.__init__(self, port)
    RoomHubMixin.__init__(self)

    # (angle, dist) pair
    self.r3d2_objects = [None, None]
    self.cake_completed = None

  def wait(self, dt):
    """Wait for a given period, in seconds"""
    t = time.time() + dt
    while time.time() < t:
      self.run_one()

  def killall(self):
    frame = Frame(self.address, 0xFF, PayloadSystemStop(1))
    #TODO send it multiple times?
    self.send(frame)

  def payload_handler_room(self, frame):
    pl = frame.payload

    # Events
    if isinstance(pl.transaction, RoomEvent):
      params = pl.params
      # R3D2 detection
      if pl.mname == 'r3d2_detected':
        self.r3d2_objects[params.i] = (params.a, params.r)
      elif pl.mname == 'r3d2_disappeared':
        self.r3d2_objects[params.i] = None
      elif pl.mname == 'meca_cake_completed':
        self.cake_completed = True
    else:
      RoomHubMixin.payload_handler_room(self, frame)


def led_set(n, v):
  """Set state of a Hreidmar led"""
  open('/dev/hreidmar/ledD%d'%n, 'wb').write('1' if v else '0')

def leds_sequence_iter(states):
  """Iterator to set successive led states"""
  while True:
    for s in states:
      for i,c in enumerate(s):
        if c == '0':
          led_set(i, False)
        elif c == '1':
          led_set(i, True)
      yield


class Match(object):

  match_timer_duration = 90-1
  # distance under which we stop to avoid the opponent
  r3d2_avoid_distance = 0.6

  def __init__(self, hub):
    self.color = None
    self.timer_thread = None
    self.hub = hub

  def start(self):
    def signal_handler(signum, exc):
      raise Exception("signal %d" % signum)
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    self.robot_init()
    self.wait_start()
    self.start_timer_thread()
    self.run()
    print "end of run()"
    self.hub.killall()


  def starting_cord_plugged(self):
    """Return whether the starting cord is plugged or not"""
    v = open('/dev/hreidmar/starting_cord').read().strip()
    return not bool(int(v))

  def robot_init(self):
    hub = self.hub
    self.set_arm_mode('close')

    hub.send_room_wait(addrs.prop, room.galipeur_set_a_speed(600, 100))
    hub.send_room_wait(addrs.prop, room.galipeur_set_xy_cruise_speed(10, 0.05))
    hub.send_room_wait(addrs.prop, room.galipeur_set_xy_steering_speed(5, 0.05))
    hub.send_room_wait(addrs.prop, room.galipeur_set_xy_stop_speed(1, 0.05))
    hub.send_room_wait(addrs.prop, room.galipeur_set_windows(50, 5, 0.03))

    hub.send_room_wait(addrs.r3d2, room.r3d2_set_conf(4000, 50, 0.573, 11))

  def wait_start(self):
    """Wait for the beginning of the match"""
    for i in (0, 1, 2, 3, 4, 8, 12):
      led_set(i, 0)
    if not self.starting_cord_plugged():
      print "please, plug the starting cord"
      led_it = leds_sequence_iter(['---1', '---0'])
      while not self.starting_cord_plugged():
        next(led_it)
        self.hub.run_one(0.5)
    print "waiting for starting cord..."
    led_set(3, 0)
    led_set(4, 1)
    while self.starting_cord_plugged():
      self.hub.run_one(0.1)
      color2 = self.get_color()
      if color2 != self.color:
        self.color = color2
        print "changing color to %s ..." % color2name[self.color]
        self.hub.send_room_wait(addrs.meca, room.robot_color(self.color))
        print "ready"
    self.color = self.get_color()
    print "start match with color %s" % color2name[self.color]

  def start_timer_thread(self):
    def match_timer():
      import os
      time.sleep(self.match_timer_duration)
      led_set(0, 0)
      led_set(1, 0)
      print "blow up balloon"
      self.hub.send_room_wait(addrs.meca, room.meca_balloon_tap(1))
      print "end of match"
      self.hub.killall()
      print "stop asserv"
      self.hub.send_room(addrs.prop, room.asserv_activate(False))
      hub.send_room(addrs.prop, room.galipeur_force_thrust(0, 0, 0))
      print "stop meca"
      #self.hub.send_room(addrs.meca, room.meca_set_arm_mode(0))

      os._exit(0)

    self.timer_thread = threading.Thread(target=match_timer)
    self.timer_thread.daemon = True
    print "start match timer thread"
    self.timer_thread.start()

  @classmethod
  def get_color(self):
    """Get color from the color switch"""
    v = open('/dev/hreidmar/color_select').read().strip()
    v = bool(int(v))
    ret = COLOR_RED if v else COLOR_BLUE
    if ret == COLOR_RED:
      led_set(0, 0)
      led_set(1, 1)
    else:
      led_set(0, 1)
      led_set(1, 0)
    return ret


  def set_arm_mode(self, mode):
    mode = {
        'home': 0,
        'close': 0,
        'stalling': 1,
        'open': 1,
        'caking': 2,
        }.get(mode, 0)
    self.hub.send_room_wait(addrs.meca, room.meca_set_arm_mode(mode))


  def opponent_detected(self):
    """Return True if an opponent is detected"""
    return any(o is not None and o[1] < self.r3d2_avoid_distance for o in self.hub.r3d2_objects)


  def goto_xya(self, x, y, a=None):
    """Go to x,y,a position, avoid opponents"""
    hub = self.hub
    def do_goto():
      print "do goto"
      hub.send_room_wait(addrs.prop, room.asserv_goto_xy(x, y))
      if a is not None:
        hub.send_room_wait(addrs.prop, room.asserv_goto_a(a))

    status_l = []  # nonlocal is not available :(
    def cb_status(frame):
      status_l.append(frame.payload)

    do_goto()
    hub.send_room(addrs.prop, room.asserv_status(), cb_status)
    while True:
      if len(status_l):
        # check returned asserv status
        r = status_l.pop()
        if r.params.arrived_xy and (a is None or r.params.arrived_a):
          return
        else:
          hub.send_room(addrs.prop, room.asserv_status(), cb_status)

      # stop when r3d2 detects something close
      if self.opponent_detected():
        print "opponent detected at %r" % (hub.r3d2_objects[0],)
        hub.send_room_wait(addrs.prop, room.asserv_goto_xy_rel(0, 0))
        #hub.send_room(addrs.prop, room.asserv_activate(False))
        while self.opponent_detected():
          hub.run_one()
        print "opponent moved away"
        #hub.send_room_wait(addrs.prop, room.asserv_activate(True))
        do_goto()

      hub.run_one()

  def thrusting(self, d, a, omegaz, caking=False):
    hub = self.hub
    pl = room.galipeur_force_thrust(d*math.cos(a), d*math.sin(a), omegaz)
    hub.send_room_wait(addrs.prop, pl)
    if not caking:
      return

    hub.cake_completed = False
    while not hub.cake_completed:
      # stop when r3d2 detects something close
      if self.opponent_detected():
        print "opponent detected at: %r" % (hub.r3d2_objects,)
        hub.send_room_wait(addrs.prop, room.galipeur_force_thrust(0, 0, 0))
        while self.opponent_detected():
          hub.run_one()
        print "opponent moved away"
        hub.send_room_wait(addrs.prop, pl)
      hub.run_one()


  def run(self):
    """Main strat routine"""
    hub = self.hub

    if self.color == COLOR_BLUE:
      self.kx = -1
      x0, y0, a0 = (-1.500 + 0.120, 2.000 - 0.150, pi/6)
    else:
      self.kx = 1
      x0, y0, a0 = (1.500 - 0.120, 2.000 - 0.150, pi/2)
    x0, y0, a0 = (0, 0, 0)
    hub.send_room_wait(addrs.prop, room.asserv_set_position(x0, y0, a0))
    print hub.send_room_wait(addrs.prop, room.asserv_get_position())
    #XXX
    if 0x14 in board_addrs:
      try:
        hub.send_room_wait(addrs.pmi, room.pmi_go())
      except Exception as e:
        print "no response to pmi_go: %s" % e

    self.run_caking()
    print "wait for end of match"
    while True:
      self.hub.run_one()


  def run_caking(self):
    hub = self.hub
    kx = self.kx

    if kx == 1:
      print "left the starting area"
      self.goto_xya(-0.1, 0.1)
      print "open arms"
      self.set_arm_mode('open')

      print "go near the cake "
      self.goto_xya(0, 0.8, pi/6)

      print "disable asserv"
      hub.send_room_wait(addrs.prop, room.asserv_activate(False))

      print "push the cake"
      self.thrusting(30e6, pi/6, 0)
      hub.wait(1.0)

      print "stalling..."
      self.thrusting(50e6, -math.radians(20), -1e6)
      hub.wait(1.0)

      print "change arm mode to caking"
      self.set_arm_mode('caking')
      hub.wait(0.5)
      print "go along the cake"
      self.thrusting(40e6, math.radians(70), -1e6, True)

      #print "back from the cake"
      #self.thrusting(40e6, -pi, 0)
      #hub.wait(0.5)
      #print "close arms"
      #hub.send_room_wait(addrs.prop, room.galipeur_force_thrust(0, 0, 0))
      #self.set_arm_mode('close')

      print "re-enable asserv"
      hub.send_room_wait(addrs.prop, room.asserv_activate(False))
      hub.send_room_wait(addrs.prop, room.asserv_set_position(0, 0, 0))
      hub.send_room_wait(addrs.prop, room.asserv_activate(True))

      print "go back"
      self.goto_xya(0, -0.05)
      print "close arms"
      hub.send_room_wait(addrs.prop, room.galipeur_force_thrust(0, 0, 0))
      self.set_arm_mode('close')

      print "go to glasses"
      self.goto_xya(0, -0.87, -math.radians(13))
      print "reset position"
      hub.send_room_wait(addrs.prop, room.asserv_activate(False))
      hub.send_room_wait(addrs.prop, room.asserv_set_position(0, 0, 0))
      hub.send_room_wait(addrs.prop, room.asserv_activate(True))

      print "push glasses"
      d, a = 2.05, pi/6
      self.goto_xya(d*math.cos(a), d*math.sin(a))

    else:
      print "left the starting area"
      self.goto_xya(0, -0.1)
      print "open arms"
      self.set_arm_mode('open')

      print "go near the cake "
      self.goto_xya(0.6, -0.5, -pi/6)

      print "disable asserv"
      hub.send_room_wait(addrs.prop, room.asserv_activate(False))

      print "push the cake"
      self.thrusting(30e6, pi/4, 0)
      hub.wait(1.0)

      print "stalling..."
      self.thrusting(50e6, math.radians(80), 1e6)
      hub.wait(1.0)

      print "change arm mode to caking"
      self.set_arm_mode('caking')
      hub.wait(0.5)
      print "go along the cake"
      self.thrusting(40e6, math.radians(0), 1e6, True)

      #print "back from the cake"
      #self.thrusting(40e6, -pi, 0)
      #hub.wait(0.5)
      #print "close arms"
      #hub.send_room_wait(addrs.prop, room.galipeur_force_thrust(0, 0, 0))
      #self.set_arm_mode('close')

      print "re-enable asserv"
      hub.send_room_wait(addrs.prop, room.asserv_set_position(0, 0, 0))
      hub.send_room_wait(addrs.prop, room.asserv_activate(True))

      print "go back"
      self.goto_xya(-0.05, 0)
      print "close arms"
      hub.send_room_wait(addrs.prop, room.galipeur_force_thrust(0, 0, 0))
      self.set_arm_mode('close')

      print "go to glasses"
      self.goto_xya(-0.87, 0.28, math.radians(13))
      print "reset position"
      hub.send_room_wait(addrs.prop, room.asserv_activate(False))
      hub.send_room_wait(addrs.prop, room.asserv_set_position(0, 0, 0))
      hub.send_room_wait(addrs.prop, room.asserv_activate(True))

      print "push glasses"
      d, a = 2.05, pi/6
      self.goto_xya(d*math.cos(a), d*math.sin(a))

      print "go to first gift 1"
      d, a = 1.80, pi/6
      x, y = d*math.cos(a), d*math.sin(a)
      self.goto_xya(x, y)
      print "go to first gift 2"
      d, a = 0.9, pi/6+pi/2
      x, y = x + d*math.cos(a), y + d*math.sin(a)
      self.goto_xya(x, y, pi/2)

      print "disable asserv"
      hub.send_room_wait(addrs.prop, room.asserv_activate(False))


def main():
  import argparse

  parser = argparse.ArgumentParser()
  parser.add_argument('port', type=int, nargs='?',
      help="TCP server port")
  args = parser.parse_args()

  hub = StratHub(args.port)
  hub.start()
  match = Match(hub)
  try:
    match.start()
  except Exception:
    hub.killall()
  finally:
    traceback.print_exc()
    hub.stop()


if __name__ == '__main__':
  main()


