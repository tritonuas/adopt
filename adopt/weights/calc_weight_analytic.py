from adopt.weights.fuse_weight_analytic_model import FuseWeightAnalyticModel
from adopt.weights.hs_weight_analytic_model import HsWeightAnalyticModel
from adopt.weights.tail_boom-weight_analytic_model import TailBoomWeightAnalyticModel
from adopt.weights.vs_weight_analytic_model import VsWeightAnalyticModel
from adopt.weight.wing_weight_analytic_model import WingWeightAnalyticModel



wing_weight = 
horizontal_stabilizer_weight = 
vertical_stabilizer_weight = 
fuselage_weight = 
tail_boom_weight = 
gross = wing_weight + horizontal_stabilizer_weight + vertical_stabilizer_weight + fuselage_weight + tail_boom_weight + battery + payload_weight
gross = gross * fudge_factor

