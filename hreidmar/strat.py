#!/usr/bin/env python2.7
import sys
import os
# expand PYTHONPATH to be able to find everything
_this_dir = os.path.abspath(os.path.dirname(__file__))
sys.path.append(_this_dir)
sys.path.append(os.path.dirname(_this_dir))

from pppcommon import *
from serverhub import HreidmarHub

COLOR_RED = 1
COLOR_BLUE = 2
color2name = {
    COLOR_RED: 'red',
    COLOR_BLUE: 'blue',
    }

pi = math.pi


class StratHub(HreidmarHub, RoomHubMixin):

  def __init__(self, port=None):
    HreidmarHub.__init__(self, port)
    RoomHubMixin.__init__(self)

  def wait(self, dt):
    """Wait for a given period, in seconds"""
    t = time.time() + dt
    while time.time() < t:
      hub.run_one(1)

  def killall(self):
    frame = Frame(self.address, 0xFF, PayloadSystemStop(1))
    #TODO send it multiple times?
    self.send(frame)


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
    return False #TODO
    v = open('/dev/hreidmar/starting_cord').read().strip()
    return not bool(int(v))


  def robot_init(self):
    #TODO close arms
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
        print "changed color to %s" % color2name[self.color]
    self.color = self.get_color()
    print "start match with color %s" % color2name[self.color]

  def start_timer_thread(self):
    def match_timer():
      import os
      time.sleep(self.match_timer_duration)
      led_set(0, 0)
      led_set(1, 0)
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
    return COLOR_RED
    #TODO
    v = open('/dev/hreidmar/color_select').read().strip()
    v = bool(int(v))
    ret = COLOR_RED if v else COLOR_BLUE
    if ret == COLOR_BLUE:
      led_set(0, 0)
      led_set(1, 1)
    else:
      led_set(0, 1)
      led_set(1, 0)
    return ret


  def run(self):
    """Main strat routine"""
    hub = self.hub


def main():
  import argparse

  parser = argparse.ArgumentParser()
  parser.add_argument('port', type=int,
      help="TCP server port")
  args = parser.parse_args()

  hub = HreidmarHub(args.port)
  match = Match()
  try:
    match.start(hub)
  except Exception:
    hub.killall()
  finally:
    hub.stop()


if __name__ == '__main__':
  main()


