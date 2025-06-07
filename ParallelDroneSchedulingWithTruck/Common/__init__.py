# -*- coding: utf-8 -*-
# @Time     : 2025-05-08-10:06
# @Author   : Sum3 TEO
# @E-mail   : rui3zhang@163
from math import *
import re


class Route(object):
    def __init__(self,
                 truck_or_drone=0,
                 route_list=None,
                 time_list=None,
                 distance=0,
                 satis=None):
        self.truck_or_drone = truck_or_drone  # 0: truck, 1: drone
        self.routeList = route_list
        self.timeList = time_list
        self.distance = distance
        self.satisfaction = satis


def calc_route_dis(ins,
                   route_list,
                   truck_or_drone):
    dis = 0
    for idx in range(len(route_list) - 1):
        if truck_or_drone == 0:
            dis += ins.graph.arcDict[route_list[idx], route_list[idx + 1]].mDistance
        else:
            dis += ins.graph.arcDict[route_list[idx], route_list[idx + 1]].eDistance

    return dis

def clac_euclidean_dis(x_1, y_1,
                       x_2, y_2):
    """ calc euclidean distance """
    return round(sqrt((x_1 - x_2) ** 2 + (y_1 - y_2) ** 2), 2)


def clac_manhattan_dis(x_1, y_1,
                       x_2, y_2):
    """ calc manhattan distance """
    return fabs((x_1 - x_2)) + fabs((y_1 - y_2))


def calc_to_ld_power_consumption(para,
                                 a_w,
                                 v,
                                 t):
    """
    c-drone takeoff and landing power consumption
    :param para: parameters
    :param a_w: average weight
    :param v: takeoff or landing speed
    :param t: travel time
    :return:
    """
    p = para.k_1 * (para.w_drone + para.w_battery + a_w) * para.gravity * (v / 2 + sqrt(
        (v / 2) ** 2 + ((para.w_drone + para.w_battery + a_w) * para.gravity) / para.k_2 ** 2)) + para.c_2 * (
                (para.w_drone + para.w_battery + a_w) * para.gravity) ** (3 / 2)
    return round(p * t, 2)


def calc_cruise_power_consumption(para,
                                  a_w,
                                  t):
    """
    c-drone cruise power consumption
    :param para: parameters
    :param a_w: average weight
    :param t: travel time
    :return:
    """
    p = (para.c_1 + para.c_2) * (((para.w_drone + para.w_battery + a_w) * para.gravity - para.c_5 * (
            para.droneCruiseSpeed * cos(para.alpha)) ** 2) ** 2) ** (3 / 4) + (para.c_4 * (para.droneCruiseSpeed ** 3))

    return round(p * t, 2)


def print_data(instance):
    """
    This function is to print the instance info.
    :param instance: a data instance.
    :return:
    """
    print('-------  DataStructures Info --------')
    print('truck number: {}'.format(instance.truckNum))
    print('drone number: {}'.format(instance.droneNum))
    print('customer number: {}'.format(instance.cusNum))
    print('node number: {}'.format(instance.cusNum + 2))
    print('arc number: {}'.format(len(instance.graph.arcDict)))
    print('demand\th ready time\ts ready time\ts due time\th due time')
    for i in instance.graph.vertexDict.keys():
        print('{}\t\t{}\t\t{}\t\t{}\t\t{}'.format(instance.graph.vertexDict[i].demand,
                                                  instance.graph.vertexDict[i].hardReadyTime,
                                                  instance.graph.vertexDict[i].softReadyTime,
                                                  instance.graph.vertexDict[i].softDueTime,
                                                  instance.graph.vertexDict[i].hardDueTime))

    print('-------  distance matrix --------')
    for i in range(instance.cusNum + 2):
        for j in range(instance.cusNum + 2):
            if (i, j) in instance.graph.arcDict.keys():
                # print("%6.2f%1.2f" % instance.graph.arcDict[i, j].eDistance, instance.graph.arcDict[i, j].mDistance,
                #       end='')
                print(f"{instance.graph.arcDict[i, j].eDistance:6.2f}{instance.graph.arcDict[i, j].mDistance:6.2f}",
                      end='')


def create_file_dir(dir_):
    """
    This function is to create dirs if the dirs is not exist.

    :param dir_:
    :return:None.
    """
    import os
    # 因为os.getcwd()是得到当前工作目录，
    # 但是我们需要的格式是 path = "E:/ly" 这样的
    current_file_dir = os.getcwd().replace('\\', '/')

    # 为了处理多级目录
    dir_arr = dir_.split('/')
    for single_dir in dir_arr:
        current_file_dir += '/' + single_dir

        # 创建工作目录
        if not os.path.exists(current_file_dir):
            os.mkdir(current_file_dir)


def get_truck_and_drone_routes(grb,
                               ins):
    routes_truck = dict()
    sol_info_truck = dict()

    routes_drone = dict()
    sol_info_drone = dict()
    for x in grb.X.keys():
        if grb.X[x].x >= 0.95:
            string = grb.X[x].varName
            idx = re.findall(r'\d+', string)

            if int(idx[-1]) not in list(sol_info_truck):
                sol_info_truck[int(idx[-1])] = [(int(idx[0]), int(idx[1]))]
            else:
                sol_info_truck[int(idx[-1])].append((int(idx[0]), int(idx[1])))

    for xi in grb.Xi.keys():
        if grb.Xi[xi].x >= 0.95:
            string = grb.Xi[xi].varName
            idx = re.findall(r'\d+', string)

            if int(idx[-1]) not in list(sol_info_drone):
                sol_info_drone[int(idx[-1])] = [(int(idx[0]), int(idx[1]))]
            else:
                sol_info_drone[int(idx[-1])].append((int(idx[0]), int(idx[1])))

    depot_return = ins.cusNum + 1

    for truck in sol_info_truck:
        cur_arcs = sol_info_truck[truck]
        cur_vertex = 0
        route = [cur_vertex]

        while cur_vertex != depot_return:
            for arc in cur_arcs:
                if arc[0] == cur_vertex:
                    route.append(arc[1])
                    cur_vertex = arc[1]

        routes_truck[truck] = route

    for drone in sol_info_drone:
        depot_return = ins.cusNum + 1
        cur_arcs = sol_info_drone[drone]
        cur_vertex = 0
        route = [cur_vertex]

        for i in range(1, ins.cusNum + 1):
            for t in range(ins.truckNum):
                if grb.Z[i, drone, t].x >= 0.95:
                    depot_return = i

        while cur_vertex != depot_return:
            for arc in cur_arcs:
                if arc[0] == cur_vertex:
                    route.append(arc[1])
                    cur_vertex = arc[1]

        routes_drone[drone] = route

    # check
    del_truck = []
    for truck, route in routes_truck.items():
        if len(route) == 2:
            del_truck.append(truck)
    for truck in del_truck:
        routes_truck.pop(truck)
    del_drone = []
    for drone, route in routes_drone.items():
        if len(route) == 2 and route[-1] == ins.cusNum + 1:
            del_drone.append(drone)
    for drone in del_drone:
        routes_drone.pop(drone)
    return routes_truck, routes_drone


def get_serve_time(grb,
                   ins,
                   routes_truck: dict,
                   routes_drone: dict):
    serve_time_truck = dict()
    serve_time_drone = dict()
    distances_truck = dict()
    distances_drone = dict()
    satisfaction = dict()
    for truck, route in routes_truck.items():
        cur_time_list = list()
        cur_dis = 0

        for vertex in route:
            cur_time_list.append(grb.Tau_t[vertex, truck].x)
            if vertex != 0 and vertex != ins.cusNum + 1:
                satisfaction[vertex] = calc_satisfaction(ins=ins,
                                                         cus=vertex,
                                                         serve_time=grb.Tau_t[vertex, truck].x)

            if vertex != route[-1]:
                cur_idx = route.index(vertex)
                cur_dis += ins.graph.arcDict[vertex, route[cur_idx + 1]].mDistance

        serve_time_truck[truck] = cur_time_list
        distances_truck[truck] = cur_dis

    for drone, route in routes_drone.items():
        cur_time_list = list()
        cur_dis = 0

        for vertex in route:
            cur_time_list.append(grb.Tau_d[vertex, drone].x)
            if vertex != 0 and vertex != ins.cusNum + 1:
                satisfaction[vertex] = calc_satisfaction(ins=ins,
                                                         cus=vertex,
                                                         serve_time=grb.Tau_d[vertex, drone].x)

            if vertex != route[-1]:
                cur_idx = route.index(vertex)
                cur_dis += ins.graph.arcDict[vertex, route[cur_idx + 1]].eDistance

        serve_time_drone[drone] = cur_time_list
        distances_drone[drone] = cur_dis

    return serve_time_truck, serve_time_drone, distances_truck, distances_drone, satisfaction


def calc_satisfaction(ins,
                      cus,
                      serve_time):
    if ins.graph.vertexDict[cus].hardReadyTime <= serve_time < ins.graph.vertexDict[cus].softReadyTime:
        satisfaction = (serve_time - ins.graph.vertexDict[cus].hardReadyTime) / (
                ins.graph.vertexDict[cus].softReadyTime - ins.graph.vertexDict[cus].hardReadyTime)

    elif ins.graph.vertexDict[cus].softReadyTime <= serve_time <= ins.graph.vertexDict[cus].softDueTime:
        satisfaction = 1
    else:
        satisfaction = (ins.graph.vertexDict[cus].hardDueTime - serve_time) / (
                ins.graph.vertexDict[cus].hardDueTime - ins.graph.vertexDict[cus].softDueTime)

    return satisfaction


def get_sol_routes(grb,
                   ins):
    routes_truck, routes_drone = get_truck_and_drone_routes(grb=grb,
                                                            ins=ins)
    serve_time_truck, serve_time_drone, dist_truck, dist_drone, satis = get_serve_time(grb=grb,
                                                                                       ins=ins,
                                                                                       routes_truck=routes_truck,
                                                                                       routes_drone=routes_drone)
    routes_t = dict()
    routes_d = dict()

    for truck, route in routes_truck.items():
        cur_satis = 0
        for ver in route[1: -1]:
            cur_satis += satis[ver]
        new_route = Route(truck_or_drone=0,
                          route_list=route,
                          time_list=serve_time_truck[truck],
                          distance=dist_truck[truck],
                          satis=cur_satis
                          )
        routes_t[truck] = new_route

    for drone, route in routes_drone.items():
        cur_satis = 0
        for ver in route[1: -1]:
            cur_satis += satis[ver]
        new_route = Route(truck_or_drone=0,
                          route_list=route,
                          time_list=serve_time_drone[drone],
                          distance=dist_drone[drone],
                          satis=cur_satis
                          )
        routes_d[drone] = new_route

    return routes_t, routes_d