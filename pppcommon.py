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
import time  # for time()/sleep()
import math

import os
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


