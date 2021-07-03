import omtools.api as ot
import openmdao.api as om
import numpy as np


from calc_gross_weight import CalcGrossWeight
from parasite_drag_approx import ParasiteDragApprox
from oswald_eff_approx import OswaldEffApprox
from calc_opt_drag_polar import CalcOptDragPolar
from velocity_coupling_group import VelocityCouplingGroup

class GrossWeightCouplingGroup(om.Group):
  
  def setup(self):

    gross_weight_comp = CalcGrossWeight()
    self.add_subsystem('gross_weight_comp', gross_weight_comp, promotes=['*'])

    # Approximate oswald efficiency factor
    oswald_eff_approx_comp = OswaldEffApprox()
    self.add_subsystem('oswald_eff_approx_comp', oswald_eff_approx_comp, promotes=['*'])

    # Above two groups in a group
    velocity_coupling_group = VelocityCouplingGroup()
    self.add_subsystem('velocity_coupling_group', velocity_coupling_group, promotes=['*'])

    # self.nonlinear_solver = om.NewtonSolver(solve_subsystems=True)
    self.nonlinear_solver = om.NonlinearBlockGS()
    # self.nonlinear_solver = om.NonlinearBlockJac()
    self.nonlinear_solver.options['iprint'] = 0
    self.nonlinear_solver.options['maxiter'] = 20
    self.linear_solver = om.DirectSolver()
