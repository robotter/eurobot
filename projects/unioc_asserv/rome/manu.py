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

    @classmethod
    def log_strtime(cls):
      t = datetime.datetime.now()
      return '%02d:%02d:%02d.%03d' % (t.hour, t.minute, t.second, t.microsecond/1000)

    def on_frame(self, frame):
      if frame.msg.name in ('asserv_tm_xya'):
        print "%s <<< %r\n" % (self.log_strtime(), frame)

    def on_sent_frame(self, frame):
      pass

  client = ClientStrat(fo)

  client.start()
  try:
    import tty,termios
 
    while 1:
      fd = sys.stdin.fileno()
      os = termios.tcgetattr(fd)
      try:
        tty.setraw(sys.stdin.fileno())
        c = sys.stdin.read(1)
      finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, os)

      dx,dy,da = {
      'q':(-100,0,0),
      'd':(+100,0,0),
      'z':(0,+100,0),
      's':(0,-100,0),
      'a':(0,0,+0.4),
      'e':(0,0,-0.4)
      }.get(c,(0,0,0))
      
      client.send(Frame('asserv_goto_xy_rel', dx,dy,da))

  except KeyboardInterrupt:
    pass
  finally:
    client.stop()

if __name__ == '__main__':
  main()

