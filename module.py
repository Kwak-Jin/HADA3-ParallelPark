import pandas as pd
import numpy as np
import time
import math

# ----------------------------------------------------------

import ctypes as ct
import ctypes.wintypes as wt

# ----------------------------------------------------------

import cv2 as cv

# ----------------------------------------------------------

import matplotlib.pyplot as plt
import drawnow
import csv

# ----------------------------------------------------------

import sys

from math import *
from sympy import symbols, solve, sqrt

from multiprocessing import shared_memory
from sklearn.cluster import DBSCAN


class SHAREDMEM:
    def __init__(self):
        self.from_morai_to_main = "MORAI_TO_MAIN"
        self.from_main_to_morai = "MAIN_TO_MORAI"
        self.from_cam_to_delivry = (
            "CAM_TO_DELIVERY"  # camera deep learning process to lidar
        )
        self.from_deliv_to_cam = "DELIVRY_TO_CAM"  # lidar process to camera


class UNIT:
    def __init__(self):
        self.D2R = pi / 180
        self.R2D = 180 / pi
        self.KM2MS = 0.27777  # km/h  to  m/s

        self.LAT2METER = 110950.59672489
        self.LON2METER = 88732.3577032982

    def RAD2DEG(self, val):
        return val * self.R2D

    def DEG2RAD(self, val):
        return val * self.D2R


class REALTIME:
    def __init__(self):
        self.time_curr = 0
        self.time_cnt = 0
        self.time_stime = 0
        self.time_final = 5000
        self.time_ts = 0.01


class PARALLEL:
    def __init__(self):
        self.unit = UNIT()
        self.sm = SHAREDMEM()

        self.lidar_num = 761
        self.range_max = 9.0
        self.range_min = 0.5

        self.lidar_dist = []
        self.azimuth = np.linspace(self.unit.DEG2RAD(95), self.unit.DEG2RAD(-95), 761)

        self.x_position = []
        self.y_position = []
        self.cluster_x = []
        self.cluster_y = []

        self.epsilon = 0.25
        self.minSample = 3

        self.mission_start = [37.23949742, 126.773274]

        # lat, lon information of three parking lots
        self.parking_lot = [
            [37.2394781547383, 126.773222537377],
            [37.2394402524108, 126.773191691974],
            [37.239400350000, 126.77316510000],
        ]

        # lat, lon information of three stop position before parking path planning
        self.parking_stop_pos = [
            [37.2393862467553, 126.773199775928],
            [37.2393382728122, 126.773163496880],
            [37.2392902988386, 126.773124143336],
        ]

        self.empty_idx = 0

        self.is_mission_start = False
        self.is_detect_start = False
        self.is_detect_finish = False
        self.is_ready_planning = False
        self.is_ready_parking = False
        self.is_parking_finish = False
        self.is_waiting_finish = False

        self.S = 0
        self.H = 0
        self.near_cluster_weight = 10.0

        self.x_list = 0
        self.y_list = 0

        self.delta = 0
        self.loc_thresh = 4.5
        self.cost_thresh = 45

        self.is_certain = 0

        self.check_cnt = 0
        self.path_cnt = 0
        self.park_cnt = 0
        self.print_cnt = 0

        self.gearcmd = 4  # default : drive mode
        self.velcmd = 7
        self.steercmd = 0
        self.brakecmd = 0

        self.parking_heading = -2.258591636

        self.x_list = None
        self.y_list = None
        self.Guididx = 2

        self.waiting_start_time = None

    def parallel_parking(self, latitude, longtitude, yaw, lidar_data, org_cmd):
        self.check_parallel_start(
            latitude, longtitude
        )  #     check mission start with waypoint infomrmation

        self.lidar_dist = []

        if self.is_mission_start:
            self.lidar_dist = np.copy(lidar_data)  #     lidar data receive

            self.data_preprocessing()  #     lidar data pre-processing

            self.dbScan()  #     dbscan algorithm

            self.get_space_idx(
                latitude, longtitude
            )  #     if finding empty space is finished

            self.ready_path_planning(
                latitude, longtitude
            )  #     stop car & ready for path planning  |  is ready planning : True

            self.make_trajectory(
                longtitude, latitude
            )  #     parking trajectory generation

        if self.is_ready_parking:
            self.Rimpact_Angle_steer_control(
                longtitude, latitude, yaw
            )  #     reverse gear guidance

        elif self.is_parking_finish:
            self.impact_Angle_steer_control(longtitude, latitude, yaw)

        self.control_platform(org_cmd)  #     contorl gear, vel, steer, brake

        self.print_result()  #     print algorithm state

    def control_platform(self, org_cmd):
        if self.is_mission_start and not self.is_ready_planning:
            self.gearcmd = org_cmd[0]
            self.velcmd = org_cmd[1]
            self.steercmd = org_cmd[2]
            self.brakecmd = org_cmd[3]

        if self.is_mission_start and self.is_ready_planning:
            self.gearcmd = 1
            self.velcmd = 0
            self.steercmd = 0
            self.brakecmd = 200

        if self.is_ready_parking:
            self.gearcmd = 2
            self.velcmd = 3
            self.steercmd = self.delta
            self.brakecmd = 0

        if self.is_parking_finish and not self.is_ready_parking:
            self.gearcmd = 1
            self.velcmd = 0
            self.steercmd = 0
            self.brakecmd = 200

            if time.time() - self.waiting_start_time >= 3.0:
                self.is_waiting_finish = True  # waiting for 3 seconds

            if self.is_waiting_finish:
                self.velcmd = 3
                self.steercmd = self.delta
                self.gearcmd = 4
                self.brakecmd = 0

    def check_parallel_start(self, latitude, longtitude):
        lat_lon_diff = 0

        lat_lon_diff = np.hypot(
            (latitude - self.mission_start[0]) * self.unit.LAT2METER,
            (longtitude - self.mission_start[1]) * self.unit.LON2METER,
        )

        if lat_lon_diff <= self.loc_thresh:
            self.is_mission_start = True

    # receive lidar data from simulation
    def receive_from_lidar(self, shm_name):
        try:
            shm = shared_memory.SharedMemory(name=shm_name)
            shared_data = np.ndarray((761,), dtype="double", buffer=shm.buf)

            self.lidar_dist = np.copy(shared_data)

        except FileNotFoundError:
            print("parallel parking : cannot receive lidar data from morai ...!")

    def data_preprocessing(self):
        self.x_position, self.y_position = [], []

        for Idx in range(self.lidar_num):
            if self.lidar_dist[Idx] <= self.range_min * 500:
                self.lidar_dist[Idx] = self.range_max * 500

            if self.lidar_dist[Idx] > self.range_max * 500 and Idx != 0:
                self.lidar_dist[Idx] = self.lidar_dist[Idx - 1]

            self.lidar_dist[Idx] = self.lidar_dist[Idx] / 500

            if (
                self.lidar_dist[Idx] >= self.range_min
                and self.lidar_dist[Idx] <= 0.99 * self.range_max
            ):
                self.x_position.append(self.lidar_dist[Idx] * cos(self.azimuth[Idx]))
                self.y_position.append(self.lidar_dist[Idx] * sin(self.azimuth[Idx]))

            else:
                continue

    def dbScan(self):
        self.cluster_x = []
        self.cluster_y = []

        stack = np.stack((self.x_position, self.y_position), axis=1)

        if stack.shape[0] > 0:
            cluster = DBSCAN(eps=self.epsilon, min_samples=self.minSample).fit(stack)

            labels = cluster.labels_

            clusterN = max(labels)

            for Idx in range(clusterN):
                selected_x = np.array(self.x_position)[np.array(labels) == Idx]
                selected_y = np.array(self.y_position)[np.array(labels) == Idx]

                if np.mean(selected_x) > 0.01:
                    self.cluster_x.append(np.mean(selected_x))
                    self.cluster_y.append(np.mean(selected_y))

            self.nearest_cluster_dist()

    def nearest_cluster_dist(self):
        near_dist = 0
        cost_func = 0

        if len(self.cluster_x) != 0:
            cost_val = []

            for Idx in range(len(self.cluster_x)):
                cost_val.append(abs(self.cluster_x[Idx]) + abs(self.cluster_y[Idx]))

            near_cluster_x = self.cluster_x[np.argmin(cost_val)]
            near_cluster_y = self.cluster_y[np.argmin(cost_val)]

            if near_cluster_x > 0:
                near_dist = sqrt(near_cluster_x**2 + near_cluster_y**2)

            if near_cluster_x < 1.5:
                self.is_detect_start = True

            if self.is_detect_start:
                cost_func = self.near_cluster_weight * near_dist

            if cost_func > self.cost_thresh:
                self.is_certain += 1

            if self.is_certain >= 5:
                self.is_detect_finish = True

    # find index of empty space
    def get_space_idx(self, latitude, longtitude):
        if self.is_detect_finish:
            self.check_cnt += 1  # to prevent updating parking lot index

            if self.check_cnt == 100:
                parking_diff = []

                for Idx in range(3):
                    parking_diff.append(
                        np.hypot(
                            (latitude - self.parking_lot[Idx][0]) * self.unit.LAT2METER,
                            (longtitude - self.parking_lot[Idx][1])
                            * self.unit.LON2METER,
                        )
                    )

                self.empty_idx = np.argmin(parking_diff) + 1

                # print(f"empty space index : {self.empty_idx}")

    def ready_path_planning(self, latitude, longtitude):
        if self.is_detect_finish and self.check_cnt >= 20:
            lat_lon_diff = 0

            lat_lon_diff = np.hypot(
                (latitude - self.parking_stop_pos[self.empty_idx - 1][0])
                * self.unit.LAT2METER,
                (longtitude - self.parking_stop_pos[self.empty_idx - 1][1])
                * self.unit.LON2METER,
            )

            if lat_lon_diff < self.loc_thresh:
                self.velcmd = 0
                self.brakecmd = 200

                self.is_ready_planning = True

                self.path_cnt += 1

    """
        path planning
    """

    def bisection_method(self, a, b, tolerance, max_iterations):
        if (a - sqrt((self.S / 2) ** 2 + (self.H / 2 - (self.H - a)) ** 2)) * (
            b - sqrt((self.S / 2) ** 2 + (self.H / 2 - (self.H - b)) ** 2)
        ) >= 0:
            raise ValueError(
                "Function values at a and b should have opposite signs, ensuring a unique root in the interval."
            )
            c = 3

        for _ in range(max_iterations):
            c = (a + b) / 2

            if (
                abs((c - sqrt((self.S / 2) ** 2 + (self.H / 2 - (self.H - c)) ** 2)))
                < tolerance
            ):
                return c

            if (a - sqrt((self.S / 2) ** 2 + (self.H / 2 - (self.H - a)) ** 2)) * (
                c - sqrt((self.S / 2) ** 2 + (self.H / 2 - (self.H - c)) ** 2)
            ) < 0:
                b = c

            else:
                a = c

        return c

    def make_trajectory(self, longtitude, latitude):
        self.gear = 1  # parking gear and path planning

        if self.is_ready_planning and self.path_cnt == 50:
            midpoint_x = (longtitude + self.parking_lot[self.empty_idx - 1][1]) / 2.0
            midpoint_y = (latitude + self.parking_lot[self.empty_idx - 1][0]) / 2.0

            lat_diff = (
                latitude - self.parking_lot[self.empty_idx - 1][0]
            ) * self.unit.LAT2METER
            lon_diff = (
                longtitude - self.parking_lot[self.empty_idx - 1][1]
            ) * self.unit.LON2METER

            angleP2E = atan2(lat_diff, lon_diff)  # azimuth in Radian

            L = sqrt((lat_diff) ** 2 + (lon_diff) ** 2)
            self.S = abs(L * cos(self.parking_heading - angleP2E))
            self.H = abs(L * sin(self.parking_heading - angleP2E))

            Rmin = abs(self.bisection_method(-15, 15, 0.00001, 100))
            ParaNormal = self.parking_heading + pi / 2

            O1 = np.zeros((2, 1))
            O2 = np.zeros((2, 1))

            O1[0][0] = latitude - Rmin * sin(ParaNormal) / self.unit.LAT2METER
            O1[1][0] = longtitude - Rmin * cos(ParaNormal) / self.unit.LON2METER
            O2[0][0] = (
                self.parking_lot[self.empty_idx - 1][0]
                + Rmin * sin(ParaNormal) / self.unit.LAT2METER
            )
            O2[1][0] = (
                self.parking_lot[self.empty_idx - 1][1]
                + Rmin * cos(ParaNormal) / self.unit.LON2METER
            )

            O1_2_Mid = atan2(midpoint_y - O1[0][0], midpoint_x - O1[1][0])
            alpha = acos((self.S**2 - self.H**2) / (self.S**2 + self.H**2))

            theta_1 = np.linspace(O1_2_Mid - alpha, O1_2_Mid, 4)
            theta_2 = np.linspace(ParaNormal + alpha, ParaNormal, 4)

            X_O1 = np.zeros((4, 1))
            Y_O1 = np.zeros((4, 1))
            X_O2 = np.zeros((4, 1))
            Y_O2 = np.zeros((4, 1))

            for Idx in range(4):
                X_O1[Idx][0] = O1[1][0] + Rmin * cos(theta_1[Idx]) / self.unit.LON2METER
                Y_O1[Idx][0] = O1[0][0] + Rmin * sin(theta_1[Idx]) / self.unit.LAT2METER
                X_O2[Idx][0] = O2[1][0] - Rmin * cos(theta_2[Idx]) / self.unit.LON2METER
                Y_O2[Idx][0] = O2[0][0] - Rmin * sin(theta_2[Idx]) / self.unit.LAT2METER

            X1 = X_O1.astype("float64")
            X2 = X_O2.astype("float64")
            Y1 = Y_O1.astype("float64")
            Y2 = Y_O2.astype("float64")

            x1_list = X1.flatten().tolist()
            x2_list = X2.flatten().tolist()
            y1_list = Y1.flatten().tolist()
            y2_list = Y2.flatten().tolist()

            self.x_list = x1_list + x2_list
            self.y_list = y1_list + y2_list

            # print(self.x_list)
            # print(self.y_list)

            self.is_ready_planning = False
            self.is_ready_parking = True

    def Rimpact_Angle_steer_control(self, lon, lat, yaw):
        vel = 3
        gamma = yaw * pi / 180 - pi

        dY = (self.y_list[self.Guididx] - lat) * self.unit.LAT2METER
        dX = (self.x_list[self.Guididx] - lon) * self.unit.LON2METER

        los = atan2(dY, dX)
        eta = gamma - los
        Lf = hypot(dX, dY)

        gamdotc = -3 * vel * sin(eta) / Lf
        self.delta = gamdotc * 1.4 / vel
        self.delta = min(max(self.delta, -28), 28)

        if Lf < 0.8 and (self.Guididx + 1) < len(self.x_list):
            self.Guididx = self.Guididx + 1

        if (
            self.Guididx + 1 <= len(self.x_list)
            and self.Guididx + 1 > len(self.x_list) - 4
        ):
            if abs(gamma + pi - self.parking_heading) % (2 * pi) < 0.3:
                self.park_cnt += 1
                self.velcmd = 0
                self.brakecmd = 200

            self.Guididx = self.Guididx

        if self.park_cnt >= 30:
            self.Guididx -= 2

            self.is_parking_finish = True
            self.is_ready_parking = False

            self.waiting_start_time = time.time()

    def impact_Angle_steer_control(self, lon, lat, yaw):
        vel = 3
        gamma = yaw * pi / 180

        dY = (self.y_list[self.Guididx] - lat) * self.unit.LAT2METER
        dX = (self.x_list[self.Guididx] - lon) * self.unit.LON2METER
        los = atan2(dY, dX)
        eta = gamma - los
        Lf = hypot(dX, dY)

        gamdotc = 3 * vel * sin(eta) / Lf
        self.delta = gamdotc * 1.4 / vel
        self.delta = min(max(self.delta, -28), 28)

        if Lf < 0.8 and (self.Guididx) >= 0:
            self.Guididx -= 1

        if self.Guididx == 0:
            self.is_parking_finish = False
            self.is_ready_parking = False

    def print_result(self):
        self.print_cnt += 1

        if self.print_cnt % 300 == 0:
            print("\n----------------------------------------------------")
            print(f"|   - empty space idx     : {self.empty_idx          }")
            print(f"|   - mission start flag  : {self.is_mission_start   }")
            print(f"|   - ready planning flag : {self.is_ready_planning  }")
            print(f"|   - ready parking flag  : {self.is_ready_parking   }")
            print(f"|   - parking finish flag : {self.is_parking_finish  }")
            print(f"|   - guidance idx        : {self.Guididx            }")
            print(f"|   - waiting state       : {self.is_waiting_finish  }")


class PERIPHERAL:
    def __init__(self):
        self.colorYellow = (25, 255, 255)
        self.colorWhite = (255, 255, 255)
        self.colorRed = (0, 0, 255)
        self.colorBlue = (255, 0, 0)
        self.colorGreen = (0, 255, 0)
        self.userFont = cv.FONT_HERSHEY_COMPLEX
        self.fontSize = 15


class DYNAMIC:
    def __init__(self):
        self.range_max = 9.0
        self.range_min = 0.5
        self.lidar_num = 761
        self.dist_thresh = 2.5
        self.angle_thresh = 20

        self.epsilon = 0.25
        self.minSample = 3


class STATIC:
    def __init__(self):
        self.lidar_num = 761
        self.range_max = 10
        self.range_min = 0.5

        self.unit = UNIT()
        self.azimuth = np.linspace(self.unit.DEG2RAD(95), self.unit.DEG2RAD(-95), 761)

        self.steercmd = 0

    def static_obstacle(self, lidar_dist, steercmd):
        self.lidar_dist = np.copy(lidar_dist)  #       lidar data receive

        self.data_preprocessing(self.lidar_dist)  #       lidar data preprocessing

        self.vector_force_field(steercmd)

    def data_preprocessing(self, lidar_dist):
        self.x_position, self.y_position = [], []

        for Idx in range(self.lidar_num):
            if lidar_dist[Idx] <= self.range_min * 500:
                lidar_dist[Idx] = self.range_max * 500

            if lidar_dist[Idx] > self.range_max * 500 and Idx != 0:
                lidar_dist[Idx] = lidar_dist[Idx - 1]

            lidar_dist[Idx] = lidar_dist[Idx] / 500

            self.x_position.append(lidar_dist[Idx] * cos(self.azimuth[Idx]))
            self.y_position.append(lidar_dist[Idx] * sin(self.azimuth[Idx]))

    def vector_force_field(self, steercmd):
        k_obs = 0.2
        k_car = 0.2
        weight = 0.9
        obstacle_vec = [0, 0]
        target_vec = [0, 1]
        car_vec = [0, 0]

        for Idx in range(self.lidar_num):
            if (
                self.unit.R2D * self.azimuth[Idx] > -80
                and self.unit.R2D * self.azimuth[Idx] < 80
                and self.lidar_dist[Idx] < 4
            ):
                obstacle_vec[0] += (
                    k_obs * (10 / self.lidar_dist[Idx]) * self.y_position[Idx]
                )

            elif self.lidar_dist[Idx] < 4:
                obstacle_vec[0] += (
                    k_obs * (5 / self.lidar_dist[Idx]) * self.y_position[Idx]
                )

        car_vec[0] = weight * (steercmd) + (1 - weight) * obstacle_vec[0]
        car_vec[0] = min(max(-28, car_vec[0]), 28)
        car_vec[1] = k_car * (target_vec[1] + obstacle_vec[1])

        self.steercmd = car_vec[0]


class ERP42:
    def __init__(self):
        self.velocity = 5
        self.steer = 0
        self.brake = 0
        self.device_num = "com4"


class CALIBRATION_WITHOUT_LENS:
    def __init__(self):
        self.unit = UNIT()
        self.Alpha = 86 * self.unit.D2R
        self.Beta = 0 * self.unit.D2R
        self.Gamma = 0 * self.unit.D2R

        self.lidar_num = 12000
        self.cam_height = 0.33
        self.cam_recede = 0.20
        self.real_recede = self.cam_recede * cos(self.Beta)
        self.focal_len = 0.00367
        self.img_width = 640
        self.img_height = 480
        self.fov_X = 58.92 * self.unit.D2R
        self.fov_Y = 58.1432 * self.unit.D2R

        self.center_X = self.img_width / 2
        self.center_Y = self.img_height / 2
        self.scaling_X = (
            self.focal_len * math.tan(0.5 * self.fov_X) / (0.5 * self.img_width)
        )
        self.scaling_Y = (
            self.focal_len * math.tan(0.5 * self.fov_Y) / (0.5 * self.img_height)
        )

        self.rotation_X = np.array(
            [
                [1, 0, 0],
                [0, np.cos(self.Alpha), -np.sin(self.Alpha)],
                [0, np.sin(self.Alpha), np.cos(self.Alpha)],
            ]
        )

        self.rotation_Y = np.array(
            [
                [np.cos(self.Beta), 0, np.sin(self.Beta)],
                [0, 1, 0],
                [-np.sin(self.Beta), 0, np.cos(self.Beta)],
            ]
        )

        self.rotation_Z = np.array(
            [
                [np.cos(self.Gamma), -np.sin(self.Gamma), 0],
                [np.sin(self.Gamma), np.cos(self.Gamma), 0],
                [0, 0, 1],
            ]
        )

        self.rotation_mat = self.rotation_Z @ self.rotation_Y @ self.rotation_X

        self.translation_mat = np.array([[0], [self.cam_height], [self.real_recede]])

        self.extrinsic_mat = np.hstack((self.rotation_mat, self.translation_mat))

        self.intrinsic_mat = np.array(
            [
                [self.focal_len / self.scaling_X, 0, self.center_X],
                [0, self.focal_len / self.scaling_Y, self.center_Y],
                [0, 0, 1],
            ]
        )


# Distorted image (by wide angle lens) calibration
class CALIBRATION_WIDE_LENS:
    def __init__(self):
        self.wide_focal_x = 370.54525668
        self.wide_focal_y = 367.47281991

        self.wide_center_x = 322.3588068
        self.wide_center_y = 230.43829046

        self.dist_coef_k1 = -0.16524716
        self.dist_coef_k2 = -0.0169815
        self.dist_coef_p1 = -0.00244594
        self.dist_coef_p2 = 0.00038361
        self.dist_coef_k3 = 0.01692057

        self.wide_intrinsic_mat = np.array(
            [
                [280.38650513, 0, 322.35003847],
                [0, 269.07183838, 226.76398398],
                [0, 0, 1],
            ]
        )

        self.wide_camera_mat = np.array(
            [
                [self.wide_focal_x, 0, self.wide_center_x],
                [0, self.wide_focal_y, self.wide_center_y],
                [0, 0, 1],
            ],
            dtype="double",
        )

        self.dist_coeffs = np.array(
            [
                self.dist_coef_k1,
                self.dist_coef_k2,
                self.dist_coef_p1,
                self.dist_coef_p2,
                self.dist_coef_k3,
            ]
        )


"""
    Considers the stop condition with (platform velocity & dist thresh value)
"""


class DELIVERY:
    def __init__(self):
        self.unit = UNIT()
        self.is_A_sign_done = False  # True or False
        self.is_deliv_control = False
        self.is_deliv_brake = False
        self.dist_thresh = 2.0  # unit : [m]
        self.deliv_velocity = 5.0  # velocity command in delivery mission

        self.start_time = None

    def sign_stop(self, distance):
        if distance <= self.dist_thresh:
            self.is_deliv_control = True

            if not self.start_time:
                self.start_time = time.time()

        else:
            self.is_deliv_control = False
            self.is_deliv_brake = False
            self.start_time = None

        if self.is_deliv_control and not self.is_deliv_brake:
            elapsed_time = time.time() - self.start_time

            time_to_brake = self.dist_thresh / (self.deliv_velocity * self.unit.KM2MS)

            if elapsed_time >= time_to_brake:
                self.is_deliv_brake = True
                self.is_A_sign_done = True  # Pickup flag update

                print("BRAKING ACTIATED ...!")
