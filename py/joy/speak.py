from subprocess import Popen, PIPE
from warnings import warn

__all__ = ['say','getEngineName']

"""
Module speak provides a simple speech synthesis interface under unix-like os-s.
It works by using the commandline program espeak.

Typical usage:
>>> from speak import say
>>> say("hello world")
>>> say("it's wonderful to have text to speech working")

If espeak is not supported, the say() function does nothing.
You can tell whether espeak should 
"""
global __SYNTH
__SYNTH = None

def _espeak_say(text):
  """
  Uses the espeak speech synthesis engine to read text aloud
  
  This function uses a single instance of espeak throughout its
  life. This subprocess is created when the first call is made.
  """
  global __SYNTH
  s = __SYNTH
  if s and s.returncode is not None:
    warn("espeak terminated with code %d" % __SYNTH.returncode )
    s=None
  if s is None:
    s = Popen( "espeak -v f3 -s 200 >/dev/null 2>/dev/null", shell=True, stdin=PIPE )
  if s is None:
    return
  s.stdin.write(text+"\n")
  __SYNTH = s
  
def _nospeak_say(text):
  """
  *** Speech synthesis is disabled ***
  
  Do you have espeak installed?
  """
  pass

def getEngineName():
  """
  Returns engine name for speech synthesis engine, or '' if speech is not
  supported.
  """
  if say == _nospeak_say:
    return ''
  return say.__name__[1:-4]

esp = Popen( "espeak -x -q okay >/dev/null 2>/dev/null", shell=True )
if esp.wait():
  say = _nospeak_say
else:
  say = _espeak_say

