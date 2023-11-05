# ParallelParking
## Module 파일

### 함수 Info
control_platform(self,org_cmd)                           : Maincode로 전달되는 조향 각, 기어, 속도, 브레이크 명령 함수

check_parallel_start(self,lattitude,longitude)           : 주차 미션을 시작하는 위치까지 왔는지, is_mission_start flag 넘겨주는 함수

receive_from_lidar(self, shm_name)                       : 라이다 정보 받는 함수

data_preprocessing(self)                                 : 라이다 정보 전처리 함수

dbScan(self)                                             : 라이다 함수

nearest_cluster_dist(self)                               : 라이다 함수

get_space_idx(self,latitude,longitude)                   : 기존에 알고 있는, 주차 정보와 비교해 어떠한 주차칸에 해당하는지, idx 산출하는 함수

ready_path_planning(self, lattitude,longitude)           : is_ready_planning flag 넘겨주는 함수, stop_pos와 가까워지면 발동

bisection_method(self, a, b, tolerance, max_iterations)  : 비선형 함수 Solver, 

make_trajectory(self, longtitude, latitude)              : 궤적 생성 함수 x_list,y_list로 궤적 도출한다. 궤적 생성 후 후진 flag(is_ready_parking) 넘김

Rimpact_Angle_steer_control(self, lon, lat, yaw)         : 현재 PNG, 후진으로 Impact Angle_steer_control

impact_Angle_steer_control(self, lon, lat, yaw)          : 현재 PNG, 전진으로 Parking 후, 정상 주행을 하기 위해, 주차공간 탈출

### Position Info

Parking시에 주차 지점          : self.parking_lot

Parking시에 멈춰야하는 지점     : self.parking_stop_pos

### Flag Info
self.is_mission_start : check_parallel_start에서 발효되는 flag로 평행 주차 공간에 대하여 인지를 할지 안할지(Lidar On/Off)에 대한 flag

self.is_detect_start  : 라이다 ON

self.is_detect_finish : 라이다 OFF

self.is_ready_planning: 궤적 생성을 하기 위한 flag, 한번만 적용됌

self.is_ready_parking : 궤적이 생성된 뒤, 기어 R단으로 넣고, 평행주차를 하는 flag

self.is_parking_finish: 주차 공간에 주차한 뒤, 다시 기어를 D단으로 변경하는 flag

self.is_waiting_finish: 3초

### Variable Info 

self.parking_heading : 주차 공간에 대하여 맞춰야하는 절대 yaw값.

self.S               : 절대 heading 기준 동체의 위치와 parking 위치에 대한 종방향 거리

self.H               : 절대 heading 기준 동체의 위치와 parking 위치에 대한 횡방향 거리

self.x_list          : self.S, self.H에 대하여 계산되고 산출된 궤적의 longitude

self.y_list          : self.S, self.H에 대하여 계산되고 산출된 궤적의 lattitude

self.Guididx         : Trajectory가 산출되었을 때, x_list,y_list에 대한 몇번째 인덱스 번호

self.delta           : Guidance로 산출된 조향 명령

self.loc_thresh      : 라이다 정보로 산출된 self.parking_stop_pos(빈공간)에 대하여 멈춰야하는 지점/ self.loc_thresh만큼 stop_pos에 가까워졌다면 stop한
다.

self.check_cnt       : detect finish 후 산출된 주차 공간이 업데이트 되는 것 방지

self.path_cnt        : parking_stop_pos idx에 가까워졌을 때 주차 기어를 넣기전에 충분한 브레이

self.park_cnt        : Parking 상태에서 3초 유지를 위한 

self.print_cnt       : 3초에 한번 씩, 배치 파일에서 결과 프린트

## 현재 발생할 수 있는 문제점

### (make_trajectory)에서 생성된 곡률 반지름이 이동체의 최소 곡률 반경보다 작을 경우
이 경우, 최소 곡률을 따라 움직이지만, 해당 곡률을 추종하지 못해서 발산할 염려있음


### 너무 먼 곳에서 멈추면 Rubber cone을 들이박을수도 있음.

![image](https://github.com/JinKwak1/ParallelParking/assets/110021151/461071c7-a17d-48de-ba21-508b6a41bded)

해당 그림과 같이, 너무 먼 곳에서 멈추게 되면, 그 앞에 있는 rubber cone을 박을 수 있기 때문에, 적절한 위치에서 멈추는 것이 중요하다.

이를 해결하기 위해 적절한 loc_thresh가 필요하다.

### 후진, 전진 유도 시, 흔들림(Chattering)
현 동체가 추종하는 WP는 크게 잡아도 7미터 가로 4미터 세로 사각형 내, 10개를 추종하게 된다. 상대거리가 작기 때문에, Heading이 흔들리는 경우가 계속 발생하게 되는데, 이를 최대한 줄이기 위한 방법 또한 생각해볼만 하다.

## 해결 방안

### 필요 시에 해당 Idx에 해당하는 가장 안전하게 도는 궤적을 설정하여, 그 궤적을 항상 추종하게 만든다. 

### http://map.esran.com
위의 주소에서 위경도를 왜곡 없이 그 값을 정확하게 받아올 수 있다. 지금 너무 깊은 곳으로 주차를 하는데, 조금 더 바깥쪽으로 WP를 수정하여, 궤적을 더 얕게 조정할 수 있었다. 이로써 앞에 있는 러버콘과 부딪힐 확률이 감소하게 되었다.

