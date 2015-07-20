try:
  import pygame.locals as constants
  from pygame.locals import *
  IMPL = "True"
except ImportError: # pygame import failed
  from sys import stderr, path
  stderr.write("*** pygame missing; using compatibility wrapper instead\n")
  IMPL = "False"
  import constants_x86_64
  from constants_x86_64 import *

