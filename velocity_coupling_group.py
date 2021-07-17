import omtools.api as ot
import openmdao.api as om
import numpy as np

from parasite_drag_approx import ParasiteDragApprox
from calc_opt_drag_polar import CalcOptDragPolar

class VelocityCouplingGroup(om.Group):
  
  def setup(self):

    self.add_subsystem('parasite_drag_comp', ParasiteDragApprox(), promotes=['*'])
    self.add_subsystem('opt_drag_polar_comp', CalcOptDragPolar(), promotes=['*'])

    # self.nonlinear_solver = om.NewtonSolver(solve_subsystems=True)
    self.nonlinear_solver = om.NonlinearBlockJac()
    self.nonlinear_solver.options['iprint'] = 0
    self.nonlinear_solver.options['maxiter'] = 15
    # self.linear_solver = om.DirectSolver()
