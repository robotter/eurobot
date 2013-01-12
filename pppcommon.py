#!/usr/bin/env python2.7

# This file define useful stuff to play with PPP.
# Import it from another script:
#   from pppcommon import *
# Run it from a IPython shell:
#   %run pppcommon.py

import perlimpinpin
from perlimpinpin import payload
from perlimpinpin.frame import Frame
from perlimpinpin.io import Connection, HubBase, HubClient
from perlimpinpin.payload.system import *
from perlimpinpin.payload.log import *
from perlimpinpin.payload.room import room_payloads
from perlimpinpin.payload.room import Event as RoomEvent
from perlimpinpin.payload.room import Order as RoomOrder
from perlimpinpin.payload.room import Command as RoomCommand
import time
import math

import os
import sys
import imp

# load transactions
imp.load_source('room_transactions', os.path.join(os.path.dirname(__file__), 'room_transactions.py'))

# Gather ROOM payloads, accessed by message name
class room:
  for cls in room_payloads.values():
    locals()[cls.mname] = cls

# Known Rob'Otter PPP addresses
class addrs:
  hub = 0x10
  prop = 0x11
  meca = 0x12
  r3d2 = 0x13
  pmi = 0x14


class RoomHubMixin(object):
  """
  Hub mixin with better ROOM handling

  Check for ROOM replies, resend ROOM orders/commands on timeout.
  Callbacks are executed in the I/O thread.

  Only one callback can be registered for a given destination and response
  message ID. Sending a new one will override the previous callback.
  A response to a previously sent similar message may be received and took into
  account.

  Attributes:
    room_timeout -- timeout in seconds for ROOM replies
    room_ntries -- number of retries for sent ROOM messages
    _room_cbs -- list of callbacks, indexed by (src, mid) of expected response

  """

  room_timeout = 0.2
  room_ntries = 20

  def __init__(self):
    self._room_cbs = {}

  def send_room(self, dst, pl, cb=None):
    """Send a ROOM message, execute a custom callback on response"""
    frame = Frame(self.address, dst, pl)
    if cb is not None and pl.response is not None:
      self._room_cbs[(dst, pl.response.mid)] = cb
      self.schedule(self.room_timeout, lambda: self._room_timeout_cb(frame, self.room_ntries))
    self.send(frame)

  def send_room_wait(self, dst, pl):
    """Send a ROOM message, wait for its response
    Return the reply payload.
    """
    if pl.response is None:
      # nothing to wait for
      self.send(dst, pl)
      return None

    l = []  # nonlocal is not available :(
    def cb(frame):
      l.append(frame.payload)
    self.send_room(dst, pl, cb)
    while not len(l):
      self.run_one()
    return l[0]

  def send_room_wait_multiple(self, dsts, pl):
    """Send a ROOM message to multiple targets, wait for their response
    Return the reply payloads as a map indexed by dst.
    """
    if pl.response is None:
      # nothing to wait for
      for dst in dsts:
        self.send(dst, pl)
      return None

    d = {}  # nonlocal is not available :(
    def cb(frame):
      l[frame.src] = frame.payload
    for dst in dsts:
      self.send_room(dst, pl, cb)
    while len(d) < len(dsts):
      self.run_one()
    return d

  def _room_timeout_cb(self, frame, n):
    """Scheduled callback to resend frame on timeout"""
    key = (frame.dst, frame.payload.response.mid)
    if key not in self._room_cbs:
      # response has been received
      return
    if n == 0:
      del self._room_cbs[(frame.dst, frame.payload.response.mid)]
      raise RuntimeError("room response not received: %r" % frame)
    self.send(frame)
    self.schedule(self.room_timeout, lambda: self._room_timeout_cb(frame, n-1))

  def payload_handler_room(self, frame):
    key = (frame.src, frame.payload.mid)
    if key in self._room_cbs:
      cb = self._room_cbs.pop(key)
      cb(frame)


