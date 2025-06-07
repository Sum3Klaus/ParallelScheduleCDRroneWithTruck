class Parameters(object):

    def __init__(self):
        self.serviceTime = 10
        self.truckSpeed = 2.5  # 16.67  #      8.33 m/s
        self.droneCruiseSpeed = 2.5 * round(6 / 8.33, 2)  # 11.11   # round(6 / 8.33)  # m/s
        self.droneTakeoffSpeed = 1  # 2.5 # m/s
        self.droneLandSpeed = 0.75  # 1.5  # m/s
        self.w_drone = 2  # (kg)
        self.w_battery = 1  # (kg)
        self.ride_time_max = 240  # drone max fly time
        self.L = 6  # max c-drone num
        self.B = 4  # * 3600 * 1000   # battery
        self.singleQ = 6  # pang
        self.m = 20
        self.droneMaxGroup = 4
        self.h = 0.1  # 100

        # cost related
        self.cost_1 = 22.4  # 单位飞机固定成本 （中通研究院 56000元 / 2500次 = 34.3元/次）
        self.cost_2 = 0.595  # 无人机单位距离旅行成本 （中通研究院 29750元 / 50000km = 0.595）

        # drone related
        self.k_1 = 0.8554
        self.k_2 = 0.3051
        self.c_1 = 2.8037
        self.c_2 = 0.3177
        self.c_3 = 1e-6
        self.c_4 = 0.0296
        self.c_5 = 0.0279
        self.c_6 = 1e-6

        self.gravity = 9.8
        self.airFluidDensity = 1.204
        self.spinningBlade = 0.0064
        self.Para_Rotor = 4
        self.alpha = 10

        self.waitingTime = 360