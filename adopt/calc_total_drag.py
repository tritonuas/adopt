import omtools.api as ot
import numpy as np

class CalcTotalDrag(ot.Group):
  
  def initialize(self):
    pass

  def setup(self):
    cD0 = self.declare_input('cD0', val=0.3)
    cDi = self.declare_input('cDi', val=0.3)
    
    cD = cD0 + cDi
    
    self.register_output('cD', cD)
