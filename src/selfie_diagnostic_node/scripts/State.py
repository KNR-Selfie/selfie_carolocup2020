#!/usr/bin/env python3
from enum import Enum

class State(Enum):
  OK = 1
  WARNING = 2
  ERROR = 3
  FATAL = 4
  UNDEFINED = 100

  def key(self):
    return self.name