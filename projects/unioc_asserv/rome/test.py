#!/usr/bin/env python

import sys
sys.path.insert(0,"../../../")
import math,rome

def main():
  import argparse

  parser = argparse.ArgumentParser()
  parser.add_argument('source', default='/dev/ttyUSB0',
      help="serial device to listen from, defaults to /dev/ttyUSB0")
  parser.add_argument('--baudrate', type=int, default=38400,
      help="serial baudrate, defaults to 38400")

  args = parser.parse_args()

  from serial import Serial
  import os
  if os.name == 'nt':
    # hack to allow KeyboardInterrupt to interrupt
    fo = Serial(args.source, args.baudrate, timeout=0.5)
    def read(n):
      while True:
        c = fo._read(n)
        if c != '':
          return c
    fo._read = fo.read
    fo.read = read
  else:
    fo = Serial(args.source, args.baudrate)

  import datetime
  import time
  from rome.frame import Frame
  from rome.client import Client

  class ClientStrat(Client):

    def __init__(self, fo):
      Client.__init__(self, fo)
      self.done_xy = False
      self.done_a = False
      self.done_autoset = False

    @classmethod
    def log_strtime(cls):
      t = datetime.datetime.now()
      return '%02d:%02d:%02d.%03d' % (t.hour, t.minute, t.second, t.microsecond/1000)

    def on_frame(self, frame):
      if frame.msg.name in ('asserv_tm_y_pid'):
        print "%s <<< %r" % (self.log_strtime(), frame)

      elif frame.msg.name in ('asserv_tm_htraj_autoset_done'):
        self.done_autoset = frame.params.b == 1

      elif frame.msg.name in ('asserv_tm_htraj_done'):
        self.done_xy = frame.params.xy == 1
        self.done_a = frame.params.a == 1


    def on_sent_frame(self, frame):
 #     print "%s >>> %r" % (self.log_strtime(), frame)
      pass

  client = ClientStrat(fo)

  client.start()
  try:
    import tty,termios
 
    client.send(Frame('asserv_set_htraj_xy_cruise', 0, 30.0, 0.2))
    client.send(Frame('asserv_set_htraj_xy_stop', 0, 0.0, 0.05))

    time.sleep(0.5)

    def goto_xyar(x,y,a):
      _goto_xya(x,y,a,'asserv_goto_xy_rel')

    def goto_xya(x,y,a):
      _goto_xya(x,y,a,'asserv_goto_xy')

    def _goto_xya(x,y,a,msg_name):
      client.send(Frame(msg_name, 0, x,y,a))
      time.sleep(0.5)
      while not client.done_xy or not client.done_a:
        time.sleep(0.5)
  
    AS_LEFT = 1
    AS_RIGHT = 2
    AS_UP = 3
    AS_DOWN = 4

    client.send(Frame('asserv_set_xya', 0, 100, 100, 0.0))
    time.sleep(0.5)

    while 1:
      print "AS DOWN"
      client.send(Frame('asserv_autoset', 0, AS_DOWN, 0.0, 0.0))
      time.sleep(0.5)
      while not client.done_autoset:
        time.sleep(0.5)
      print "AS DOWN OK"

      goto_xya(100,100,0)

      print "AS LEFT"
      client.send(Frame('asserv_autoset', 0, AS_LEFT, 0.0, 0.0))
      time.sleep(0.5)
      while not client.done_autoset:
        time.sleep(0.5)
      print "AS LEFT OK"

      goto_xya(100,100,-.5*math.pi)
      goto_xya(100,100,0)

      goto_xya(100 , 500, 0)
      goto_xya(1500, 500, 0)

      goto_xya(100,  500, 0)
      goto_xya(100,  100, 0)

  except KeyboardInterrupt:
    pass
  finally:
    time.sleep(0.5)
    client.send(Frame('asserv_goto_xy_rel',0,0,0,0))
    client.stop()

if __name__ == '__main__':
  main()

