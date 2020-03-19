from numpy import ( asarray, zeros )
from motorsim import *

class DbgMotorModel(MotorModel):
    def __init__(self):
        MotorModel.__init__(self)

    def lt4(self,n,withLbl=False):
        c = ['#ff7f0e','#2ca02c','#d62728','#9467bd','#8c564b','#e377c2',
           '#7f7f7f','#bcbd22','#17becf','#1f77b4','#20ff40']
        res = dict(
          color = c[n % len(c)],
          marker = '.' if n<=POS else ''
          )
        if withLbl:
            res['label'] = [
                '$\\tilde\\theta$','$\\omega$','$b$','$\\varepsilon$','$T$',
                '$\\tau_0$','$\\tau_1$','$\\tau_2$','$\\tau_3$','$\\tau_D$',
                '$\\theta$','$\\omega_D$','$\\theta_D$'][n]
        return res

    def show(self,idx=range(10)):
        # th, om, bl, ei, tmp
        idx = asarray(idx,int16)
        isAng = { TH, POS, GP }
        wl = True
        for t,y in zip(self.t, self.y):
            #vlines(t[0],y[:,idx].flatten().min(), y[:,idx].flatten().max(),'k')
            for n,yi in enumerate(y.T):
                if not n in idx:
                  continue
                if n in isAng:
                    plot(t,yi % (2*pi),**self.lt4(n,wl))
                else:
                    plot(t,yi,**self.lt4(n,wl))
            wl = False
        grid(1)
        legend()

def test_pos(m):
    figure()
    m.clear()
    for k in range(10):
        m.set_pos(rand()*36000)
        m.runFor(20)
    subplot(311)
    m.show([TH,POS,GP,BL])
    title("Position control")
    subplot(312)
    m.show([TMP,TQ0,TQ1,TQ2,TQ3,TQD])
    subplot(313)
    m.show([EI,OM,GV])
    figure()
    subplot(211)
    m.show([POS,GP])
    subplot(212)
    m.show([TQD])

def test_vel(m):
    m.clear()
    figure()
    m.set_pos(None)
    for k in range(10):
        m.set_rpm(rand()*25)
        m.runFor(20)
    subplot(311)
    m.show([TH,POS,GP,BL])
    title("Velocity control")
    subplot(312)
    m.show([TMP,TQ0,TQ1,TQ2,TQ3,TQD])
    subplot(313)
    m.show([EI,OM,GV])
    figure()
    subplot(211)
    m.show([OM,GV])
    subplot(212)
    m.show([TQD])

if __name__=="__main__":
    m = DbgMotorModel()
    m._ext = lambda t,th: cos(th)/3+sin(t/4)*.2
    test_vel(m)
    test_pos(m)
    figure()
    plot(m.get_t(),m.get_pos())
    title("User output")
    show()
