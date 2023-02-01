import csdl 
from python_csdl_backend import Simulator

class SumWeightModel(csdl.Model):

    def initialize(self):
        pass

    def define(self): 

        wing_weight = self.declare_variable('wing_weight', val=1000)
        horizontal_stabilizer_weight = self.declare_variable('horizontal_stabilizer_weight', val=1000)
        vertical_stabilizer_weight = self.declare_variable('vertical_stabilizer_weight', val=1000)
        fuselage_weight = self.declare_variable('fuselage_weight', val=1000)
        tail_boom_weight = self.declare_variable('tail_boom_weight', val=1000)
        battery_weight = self.declare_variable('battery_weight', val=22.2)
        payload_weight = self.declare_variable('payload_weight', val=63)
        motor_weight = self.declare_variable('motor_weight', val=9.81)
        gross_weight_fudge_factor = self.declare_variable('gross_weight_fudge_factor', val=1.)

        gross_weight = wing_weight + fuselage_weight + horizontal_stabilizer_weight + vertical_stabilizer_weight + tail_boom_weight + battery_weight + payload_weight + motor_weight
        gross_weight = gross_weight * gross_weight_fudge_factor

        self.register_output('gross_weight', gross_weight)

if __name__ == '__main__':
    gross_weight = SumWeightModel()


    sim = Simulator(gross_weight)
    sim.run()

    print('gross_weight', sim['gross_weight'])



 # gross = wing_weight + horizontal_stabilizer_weight + vertical_stabilizer_weight + fuselage_weight + tail_boom_weight + battery + payload_weight
        # gross = gross * fudge_factor