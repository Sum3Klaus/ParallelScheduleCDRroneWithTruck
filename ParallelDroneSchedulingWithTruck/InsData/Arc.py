# -*- coding: utf-8 -*-
# @Time     : 2025-05-07-10:17
# @Author   : Sum3 TEO
# @E-mail   : rui3zhang@163


class Arc(object):
    def __repr__(self):
        euclidean_info = f'euclidean distance={self.eDistance},drone adj={self.adj_drone}'
        manhattan_info = f'manhattan distance={self.mDistance},truck adj={self.adj_truck}'
        return f'arc-{self.arc},{euclidean_info},{manhattan_info}'

    def __init__(self,
                 head_vertex: int,
                 tail_vertex: int,
                 euclidean_distance: float,
                 manhattan_distance: float,
                 travel_time_truck: float,
                 travel_time_drone: float,
                 adj_truck=1,
                 adj_drone=1):
        self.head_vertex = head_vertex
        self.tail_vertex = tail_vertex
        self.arc = (self.head_vertex, self.tail_vertex)
        self.eDistance = euclidean_distance
        self.mDistance = manhattan_distance
        self.truckTravelTime = travel_time_truck
        self.droneTravelTime = travel_time_drone
        self.adj_truck = adj_truck  # whether the arc is feasible
        self.adj_drone = adj_drone