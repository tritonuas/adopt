import omtools.api as ot
import numpy as np

class CalcInducedDrag(ot.Group):
  
  def initialize(self):
    pass

  def setup(self):
    k = self.declare_input('k', val=1/(np.pi*0.9*8))
    cL =  self.declare_input('cL', val=0.5)

    cDi = k*cL**2
    
    self.register_output('cDi', cDi)
    