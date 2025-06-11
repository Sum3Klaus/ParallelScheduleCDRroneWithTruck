# -*- coding: utf-8 -*-
# @Time     : 2025-05-08-12:21
# @Author   : Sum3 TEO
# @E-mail   : rui3zhang@163
from InsData.Instance import Instance
from InsData.Paramerters import Parameters
from Common import *
from GurobiFiles.Grb import Grb

""" git push test 2025.06.11 """

filepath = 'Benchmark/solomon100/r101.txt'
para = Parameters()
ins = Instance(para=para)
ins.read_data(filename=filepath,
              cus_num=16)
print_data(instance=ins)
# for arc in ins.graph.arcDict.keys():
#     print(ins.graph.arcDict[arc])

ins.truckNum = 2
ins.truckCapacity = 85
ins.para.m = 30
ins.para.droneMaxGroup = 5

grb = Grb(ins=ins)
grb.build_model()

grb.solve_and_print()
if grb.model.solCount > 0:
    routes_t, routes_d = get_sol_routes(grb=grb,
                                        ins=ins)
grb.write_lp()
if grb.model.SolCount <= 0:
    grb.model.computeIIS()
    for c in grb.model.getConstrs():
        if c.IISConstr:
            print(c.constrName)
    grb.model.write('infeasible.ilp')
print()

"""

"""