from adopt.weights.fuse_weight_analytic_model import FuseWeightAnalyticModel
from adopt.weights.hs_weight_analytic_model import HsWeightAnalyticModel
from adopt.weights.tail_boom_weight_analytic_model import TailBoomWeightAnalyticModel
from adopt.weights.vs_weight_analytic_model import VsWeightAnalyticModel
from adopt.weights.wing_weight_analytic_model import WingWeightAnalyticModel
from adopt.weights.sum_weight_model import SumWeightModel
import csdl
from python_csdl_backend import Simulator

class AnalyticWeightModel(csdl.Model):
    def initialize(self):
        pass

    def define(self):

        fuselage_weight_analytic_model = FuseWeightAnalyticModel()
        self.add(submodel=fuselage_weight_analytic_model, name='fuse_weight_analytic_model')

        hs_weight_analytic_model = HsWeightAnalyticModel()
        self.add(submodel=hs_weight_analytic_model, name='hs_weight_analytic_model')

        tail_boom_weight_analytic_model = TailBoomWeightAnalyticModel()
        self.add(submodel=tail_boom_weight_analytic_model, name='tail_boom_analytic_model')

        vs_weight_analytic_model = VsWeightAnalyticModel()
        self.add(submodel=vs_weight_analytic_model, name='vs_weight_analytic_model')

        wing_weight_analytic_model = WingWeightAnalyticModel()
        self.add(submodel=wing_weight_analytic_model, name='wing_weight_analytic_model')

        sum_weight_model = SumWeightModel()
        self.add(submodel=sum_weight_model, name='sum_weight_model')

if __name__ == '__main__':
    analytic_weight_model = AnalyticWeightModel()

    sim = Simulator(analytic_weight_model)
    sim.run()

    print('gross_weight', sim['gross_weight'])

        # gross = wing_weight + horizontal_stabilizer_weight + vertical_stabilizer_weight + fuselage_weight + tail_boom_weight + battery + payload_weight
        # gross = gross * fudge_factor

