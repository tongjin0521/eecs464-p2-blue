import pygame as PG
from numpy import array, asarray, any, unique, arange
from numpy.random import rand


class DataWindow( object ):
  """
  Concrete class DataWindow represents a sliding window of data through time
  """
  def __init__(self,duration=1):
    self.clear()
    self.duration = duration

  def clear( self ):
    """
    Empty the window.
    """
    self.push = self._firstPush
    self.t = []
    self.data = []

  def setDuration( self, duration ):
    """
    Change the duration of data allowed in the window.
    This may discard existing data if duration was made smaller
    """
    assert duration>=0
    self.duration = duration
    if self.t:
      self._trim()

  def _trim( self ):
    """
    (private) trim any entries outside the most recent window of
    self.duration time units
    """
    t = self.t[-1]
    while t-self.t[0]>self.duration:
      self.t.pop(0)
      self.data.pop(0)

  def _firstPush( self, t, val ):
    """
    Push data at the end of the window
    INPUTS:
      t -- timestamp
      val -- data record to append / replace

    If t equals last timestamp, last entry in window is replaced. Otherwise,
    entry is appended and window is trimmed to duration limit.

    (private) first push operation on an empty window
    """
    self.t = [t]
    self.data = [val]
    self.push = self._push

  def _push( self, t, val ):
    """
    Push data at the end of the window
    INPUTS:
      t -- timestamp
      val -- data record to append / replace

    If t equals last timestamp, last entry in window is replaced. Otherwise,
    entry is appended and window is trimmed to duration limit.
    """
    if t>self.t[-1]:
      self.t.append(t)
      self.data.append(val)
      self._trim()
    elif t==self.t[-1]:
      self.data[-1] = val
    else:
      raise IndexError('Time %g < last entry %g' % (t,self.t[-1]))

  def push( self, t, val ):
    """Fake placeholder -- modified by __init__, _firstPush"""
    pass

def planBoxes( grid, bw=0.1 ):
    """
    Convenience function for creating a figure layout
    INPUT:
      grid -- ASCII art layout (see example below)
      bw -- border width in grids; either scalar or (horizontal,vertical)
    OUTPUT:
      Boxes for regions, sorted by region id chars.
      Regions marked with space, '-' or '|' are ignored
    Example:
      p='''
      xxxxx yyy
      xxxxx yyy
      xxxxx yyy

      zzzzzzzzz
      zzzzzzzzz
      '''
      box_x,box_y,box_z = planAxes(p,bw=0.5)
    """
    # Convert border to 2 entry array
    bw = asarray(bw)
    if bw.size==1:
      bw = array([bw,bw])
    else:
      assert bw.size==2
    # Split lines
    g0 = grid.strip().split("\n")
    l = max([len(r) for r in g0])
    # Pad lines to constant length
    pad = " "*l
    # Create grid of chars
    g = array([ [y for y in (x+pad)[:l]] for x in g0][::-1])
    xs = 1.0/g.shape[1]
    ys = 1.0/g.shape[0]
    lst = unique(g.flatten())
    res = []
    bx,by = bw
    for nm in lst:
      if nm in ' -|':
        continue
      ind = (g==nm)
      xi = any(ind,axis=0).ravel().nonzero()[0]
      yi = any(ind,axis=1).ravel().nonzero()[0]
      box = array((
        (xi[0]+bx)*xs,(yi[0]+by)*ys,
        xs*(xi[-1]-xi[0]+1-2*bx),ys*(yi[-1]-yi[0]+1-2*by)
      ))
      res.append(box)
    return res

class Glyph( object ):
  """
  Concrete class representing a bitmap that is placed at multiple
  locations on an image. Each Glyph has an image and a basepoint
  offset (which defaults to its center). The image is RGBA, so
  pixels may be made transparent.

  By default, a Glyph contains a blue cross of the specified size,
  with lines 3 pixels wide. With size 5, this makes a good circular
  dot.
  """
  def __init__(self, size, color=(0,0,128) ):
    """
    Create a glyph of the specified size and color.

    INPUTS:
      size -- number or pair -- size of glyph image

    ATTRIBUTES:
      .ofs -- len 2 -- offset of basepoint into image
      .img -- pygame image of size size, mode RGBA -- the Glyph itself
    """
    try:
      len(size)
    except:
      size = (size,size)
    self.img = PG.image.fromstring(
      b' ' * (size[0]*size[1]*4), size,"RGBA" )
    self.ofs = (size[0]/2, size[1]/2)
    self.img.fill((0,0,0,0))
    PG.draw.line(self.img, color, (0,self.ofs[1]),(size[0],self.ofs[1]),3)
    PG.draw.line(self.img, color, (self.ofs[0],0),(self.ofs[1],size[1]),3)

  def put( self, surf, x, y ):
    """
    Put copies of the image at the specified points on the surface

    INPUTS:
      surf -- pygame.Surface to draw on
      x,y -- sequences of equal length of x and y coordinates
    """
    sz = self.img.get_size()
    for xi,yi in zip(x,y):
      surf.blit( self.img, (xi-self.ofs[0],yi-self.ofs[1],sz[0],sz[1]))

class Text( object ):
  DEFAULT_FONT = "Arial",10
  DEFAULT_COLOR = (0,0,0)
  if PG.version.ver < "1.9.4":
    DEFAULT_BACKGROUND = (255,255,255)
  else:
    DEFAULT_BACKGROUND = None
  
  def __init__(self,s = None):
    if not PG.font.get_init():
      PG.font.init()
      if not PG.font.get_init():
        raise RuntimeError("Could not start pygame font engine")
    self.set_font(*Text.DEFAULT_FONT)
    self.set_c = self.set_color
    self.set_c(Text.DEFAULT_COLOR)
    self.set_bg = self.set_background
    self.set_bg(Text.DEFAULT_BACKGROUND)
    self.set(align=())
    self.img = None
    if s is not None:
      self.set_text(s)

  def set(self,**kw):
    """
    Set multiple attributes
    """
    for k,v in kw.items():
      sf = getattr(self,"set_"+k)
      sf(*v)
  
  def set_background(self,r,g=None,b=None):
    if r is None:
      self.bg = None
    elif g is None:
      self.bg = r
    else:
      self.bg = (r,g,b)
    
  def set_color(self,*arg):
    if len(arg) is 3:
      self.color = arg
    else:
      self.color = arg[0]
  
  def set_font(self,*arg,**kw):
    self.font = PG.font.SysFont(*arg,**kw)
  
  def set_align(self,hv=None,v=None):
    if hv is None:
      self.ofs = (0,0)
    elif v is None:
      self.ofs = hv
    else:
      self.ofs = hv,v
  
  def set_text(self,s):
    """
    Set the text this text object will display
    """
    self.img = self.font.render(str(s),True,self.color,self.bg)
  
  def put(self, surf, x, y):
    sz = self.img.get_size()
    dx = int(sz[0] * self.ofs[0])
    dy = int(sz[1] * self.ofs[1])
    surf.blit( self.img, (x-dx, y-dy, sz[0], sz[1] ))
    
class LinePlotter( object ):
  """
  Concrete class LinePlotter implements simple line plotting functionality
  (including coordinate scaling and line markers) on a box within a
  pygame.Surface
  """
  color = array([0,0,255])

  def __init__(self, surf, box = (0.,0.,1.,1.) ):
    """
    INPUTS:
      surf -- the pygame.Surface on which to plot
      box -- bounding box, in range (0,0) to (1,1)

    ATTRIBUTES (public):
      .c -- color -- line color to use; autogenerated
      .mrk -- Glyph -- line symbols; default is dot with the color .c
      .bg -- color -- background color used for clearing plot area
      .axes -- 4-list -- (xmin,ymin,width,height)
    """
    w,h = surf.get_size()
    x0,y0,ww,hh = box
    self._surf = surf.subsurface( (w*x0, h*y0, w*ww, h*hh) )
    self.c = LinePlotter.color
    self.mrk = Glyph((5,5),color=LinePlotter.color)
    LinePlotter.color = (LinePlotter.color + array([0x85,0x05,0x50])) % 255
    self.bg = (255,255,255)
    self.axes = [0.,0.,1.,1.]

  def plot( self, x, y=None ):
    """
    Plot a line with the specified points

    INPUTS:
      x -- numbers -- x coordinates, in plot(x,y) form, or y coordinates
        in plot(y) form.
      y -- numbers or None -- y coordinates of points
    """
    W,H = self._surf.get_size()
    x0,y0,xs,ys = tuple(( float(x) for x in self.axes ))
    if y is None:
      py = (asarray(x)-x0)/xs * H
      px = arange(0,W,float(W)/len(py))
    else:
      px = (asarray(x)-x0)/xs * W
      py = (asarray(y)-y0)/ys * H
    if self.mrk:
      self.mrk.put(self._surf,px,py)
    if len(px)>1:
      PG.draw.aalines( self._surf, self.c, False, tuple(zip(px,py)))

  def cla( self ):
    """
    Clear the plot area to background color
    """
    self._surf.fill(self.bg)

class AnimatedWindow( object ):
  def __init__(self, size = (640,480) ):
    PG.init()
    self.screen = PG.display.set_mode((640,480))
    self.clock = PG.time.Clock()
  
  def run(self):
    """
    Animate screen updates by calling self.update(t)
    If returns true, display the frame.
    
    Allows user to quit with escape or q
    """
    from pygame.locals import QUIT, KEYDOWN, K_ESCAPE, K_q
    while True:
      self.clock.tick(60)
      PG.event.pump()
      event = PG.event.poll()
      if event.type == QUIT:
          break
      if event.type == KEYDOWN and event.key in (K_ESCAPE,K_q):
          break
      t = PG.time.get_ticks()
      if self.update(t):
        PG.display.flip()

  def update(self,t):
    pass

  
if __name__=="__main__":
  aw = AnimatedWindow()
  bx = planBoxes('abc\ndef')
  plt = [
    [LinePlotter( aw.screen, b ),DataWindow( 2000 )]
    for b in bx
  ]
  txt = Text()
  txt.set(bg=(0,255,0), font=("Arial",24),align=(0.5,0.5))
  def update(t):
    aw.screen.fill(0)
    txt.set_text(t)
    for lp,dw in plt:
      lp.cla()
      dw.push(t,rand())
      lp.axes[3] = float(dw.duration)
      lp.plot( dw.data )
    txt.put(aw.screen, 320, 240 )
    return True
  aw.update = update
  aw.run()
