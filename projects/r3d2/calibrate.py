#!/usr/bin/env python
import math
from rome import Client, Frame


def calibrate(client, angle, dist):
  raw_input("Place at angle %.0f degrees " % math.degrees(angle))
  client.send(Frame('r3d2_calibrate_angle', 0x80, angle))
  raw_input("Place at distance %.0fmm " % dist)
  client.send(Frame('r3d2_calibrate_dist', 0x81, dist))
  print "Saving configuration"
  client.send(Frame('r3d2_conf_save', 0x82))
  print "Finished"


def main():
  import argparse

  parser = argparse.ArgumentParser()
  parser.add_argument('--baudrate', type=int, default=38400,
      help="serial baudrate, defaults to 38400")
  parser.add_argument('source', default='/dev/ttyUSB0',
      help="serial device to listen from, defaults to /dev/ttyUSB0")
  parser.add_argument('angle', type=float, default=90, nargs='?',
      help="angle used for calibration in degrees")
  parser.add_argument('dist', type=float, default=300, nargs='?',
      help="distance used for calibration in millimeters")

  args = parser.parse_args()

  from serial import Serial
  fo = Serial(args.source, args.baudrate)
  class MyClient(Client):
    def on_frame(self, frame):
      pass
  client = MyClient(fo)

  client.start()
  calibrate(client, math.radians(args.angle), args.dist)
  client.stop()


if __name__ == '__main__':
  main()

