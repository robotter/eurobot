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

  def wait(self, dt):
    """Wait for a given period, in seconds"""
    t = time.time() + dt
    while time.time() < t:
      hub.run_one(1)

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
  r3d2_avoid_distance = 0.8

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
    self.set_arm_mode('close')
    #TODO configure asserv parameters
    #TODO configure r3d2 (?)
    return

  def wait_start(self):
    """Wait for the beginning of the match"""
    for i in (0, 1, 2, 3, 4, 8, 12):
      led_set(i, 0)
    if not self.starting_cord_plugged():
      print "please, plug the starting cord"
      led_it = leds_sequence_iter(['---1', '---0'])
      while not self.starting_cord_plugged():
        next(led_it)
        time.sleep(0.5)
    print "waiting for starting cord..."
    led_set(3, 0)
    led_set(4, 1)
    while self.starting_cord_plugged():
      time.sleep(0.1)
      color2 = self.get_color()
      if color2 != self.color:
        self.color = color2
        print "changing color to %s ..." % color2name[self.color]
        self.hub.send_room(addrs.meca, room.robot_color(self.color), None)
        print "ready"
    self.color = self.get_color()
    print "start match with color %s" % color2name[self.color]

  def start_timer_thread(self):
    def match_timer():
      import os
      time.sleep(self.match_timer_duration)
      led_set(0, 0)
      led_set(1, 0)
      self.hub.send_room_wait(addrs.meca, room.meca_balloon_tap(1))
      print "end of match"
      self.hub.killall()
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
    self.hub.send_room(addrs.meca, room.meca_set_arm_mode(mode), None)


  def goto_xya(self, x, y, a=None, kx=True):
    """Go to x,y,a position, avoid opponents"""
    if kx:
      x = x * self.kx
      if a is not None and kx == -1:
        a = pi*2./3 - a
    hub = self.hub
    if a is None:
      pl_goto = room.asserv_goto_xy(x, y)
    else:
      #TODO handle a
      pl_goto = room.asserv_goto_xy(x, y)

    status_l = []  # nonlocal is not available :(
    def cb_status(frame):
      status_l.append(frame.payload)

    hub.send_room(addrs.prop, pl_goto, None)
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
      if hub.r3d2_objects[0] is not None and hub.r3d2_objects[0][1] < self.r3d2_avoid_distance:
        print "opponent detected at %r" % (hub.r3d2_objects[0],)
        pl_pos = hub.send_room_wait(addrs.prop, room.asserv_get_position())
        hub.send_room_wait(addrs.prop, room.asserv_goto_xy(pl_pos.params.x, pl_pos.params.y))
        #hub.send_room(addrs.prop, room.asserv_activate(False))
        while hub.r3d2_objects[0] is not None and hub.r3d2_objects[0][1] < self.r3d2_avoid_distance:
          hub.run_one()
        print "opponent moved away"
        #hub.send_room_wait(addrs.prop, room.asserv_activate(True))
        hub.send_room(addrs.prop, pl_goto, None)

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
    #hub.send_room_wait(addrs.prop, room.asserv_set_position(x0, y0, a0))
    try:
      hub.send_room_wait(addrs.pmi, room.pmi_go())
    except Exception as e:
      print "no response to pmi_go: %s" % e

    self.run_homologation()


  def run_homologation(self):
    hub = self.hub

    if self.kx == 1:
      self.goto_xya(-0.5, 1.0, kx=False)
    else:
      self.goto_xya(0.5, -1.0, kx=False)
    #self.goto_xya(1.500 - 0.120 - 0.500, 2.000 - 0.250, kx=True)
    print "wait for end of match"
    while True:
      hub.run_one()



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


