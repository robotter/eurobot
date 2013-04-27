#!/usr/bin/env python2.7
import threading
import SocketServer
from serial import Serial
from perlimpinpin.frame import Frame
from perlimpinpin.io import Connection, HubBase
from perlimpinpin.payload.system import *
from perlimpinpin.payload.log import *


hreidmar_addr = 0x10

hreidmar_nodes = [
  (0x11, '/dev/ttySAC1'),  # prop
  (0x12, '/dev/ttySAC2'),  # meca
  (0x13, '/dev/ttySAC3'),  # r3d2
  #(0x14, '/dev/ttyUSB0'),  # pmi
]


class HubTCPServer(SocketServer.TCPServer):
  allow_reuse_address = True

  def process_request(self, request, client_address):
    con = Connection(request)
    print "new TCP connection from %s:%d" % client_address
    hub = self.hub
    hub.add_connection(con)
    # send a ping, replies will register nodes in hub network
    con.send(Frame(hub.address, 0xFF, PayloadSystemPing(1, 1)))



class HreidmarHub(HubBase):
  """
  Hub connected to all boards, and acting as a TCP server
  """

  def __init__(self, port=None):
    HubBase.__init__(self, hreidmar_addr)
    self.board_cons = set()
    for addr, src in hreidmar_nodes:
      con = Connection(Serial(src, 38400))
      self.add_connection(con)
      self.network[addr] = con
      self.board_cons.add(addr)
    if port is not None:
      self.server = HubTCPServer(('', port), None)
      self.server_thread = threading.Thread(target=self.server.serve_forever)
      self.server_thread.daemon = True
      self.server.hub = self
    else:
      self.server is None
    self.node_name = 'hreidmarhub'

  def start(self):
    if self.server:
      print "hub listening on port %d" % args.port
      self.server_thread.start()
    HubBase.start(self)

  def stop(self):
    if self.server:
      self.server.shutdown()
    HubBase.stop(self)

  def remove_connection(self, con):
    for addr, c in self.network.items():
      if c is con:
        print "unregister address 0x%02X" % addr
        del self.network[addr]
        break
    HubBase.remove_connection(self, con)

  def route_frame(self, frame, con=None):
    # update network on ping reply
    if con is not None and frame.dst == self.address and isinstance(frame.payload, PayloadSystemPing):
      print "register address 0x%02X with connection %r" % (frame.src, con)
      self.network[frame.src] = con
    if frame.dst == 0xff:
      # hack: don't broadcast logs to boards
      if isinstance(pl, PayloadLog):
        return [c for c in self.cons if c != con and c not in self.board_cons]
      else:
        return [c for c in self.cons if c != con]
    elif frame.dst != self.address and frame.dst in self.network:
      c = self.network[frame.dst]
      if c != con:
        return [c]
    return []


def main():
  import argparse

  parser = argparse.ArgumentParser()
  parser.add_argument('port', type=int,
      help="TCP server port")
  args = parser.parse_args()

  hub = HreidmarHub(args.port)
  hub.start()
  print "hub running on port %d" % args.port
  hub.run()


if __name__ == '__main__':
  main()

