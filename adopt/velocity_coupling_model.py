import csdl
import numpy as np

from adopt.parasite_drag_model import ParasiteDragModel
from adopt.drag_polar_model import DragPolarModel

class VelocityCouplingModel(csdl.Model):
  def initialize(self):
    pass
  
  def define(self):
    parasite_drag_model = ParasiteDragModel()
    self.add(submodel=parasite_drag_model, name='parasite_drag_model')

    drag_polar_model = DragPolarModel()
    self.add(submodel=drag_polar_model, name='drag_polar_model')

    self.nonlinear_solver = csdl.NewtonSolver()
    self.nonlinear_solver = csdl.NonlinearBlockJac()
    self.linear_solver = csdl.ScipyKrylov()

    # # self.nonlinear_solver = om.NewtonSolver(solve_subsystems=True)
    # self.nonlinear_solver = om.NonlinearBlockJac()
    # self.nonlinear_solver.options['iprint'] = 0
    # self.nonlinear_solver.options['maxiter'] = 15
    # # self.linear_solver = om.DirectSolver()
