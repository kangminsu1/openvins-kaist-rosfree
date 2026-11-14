# OpenVINS KAIST Urban ROS-free runner

Ubuntu 24.04에서 ROS 없이 [OpenVINS](https://github.com/rpng/open_vins)를 이용해 KAIST Urban 데이터의 스테레오 카메라와 IMU를 처리하는 실행기입니다.

다음 두 결과를 생성합니다.

- `trajectory_openvins.csv`: 시간별 IMU 위치와 자세
- `cloud.pcd`: 스테레오 영상으로 복원한 전역 3D 포인트 클라우드

> 이 프로젝트는 OpenVINS의 VIO 추정 결과를 사용합니다. 루프 클로징이나 전역 pose-graph 최적화는 포함하지 않으므로 긴 주행에서는 누적 드리프트가 발생할 수 있습니다.

## 처리 흐름

1. Xsens IMU와 좌·우 카메라 영상을 시간순으로 OpenVINS에 입력합니다.
2. OpenVINS MSCKF가 IMU 궤적을 추정합니다.
3. 보정된 스테레오 영상에서 OpenCV StereoSGBM으로 깊이를 계산합니다.
4. 카메라 포인트를 카메라 → IMU → 전역 좌표로 변환해 PCD로 저장합니다.

ROS, rosbag 재생, RTAB-Map은 사용하지 않습니다.

## 시험 환경

- Windows 11 + WSL2
- Ubuntu 24.04
- OpenVINS commit `69488123ed9362dd44b6f28e7f4680abbff1442b`
- KAIST Urban 28 (Pangyo), 약 11.47 km
- 스테레오 영상 19,745쌍
- 결과 궤적 19,646 poses

## 사용 센서

KAIST Urban 원본 차량에는 스테레오 카메라, 2D/3D LiDAR, Xsens IMU, FOG, 휠 엔코더 및 RTK GPS가 있습니다. 이 프로그램은 다음 센서만 사용합니다.

| 센서 | 이 프로그램에서 사용 | 데이터 속도/형식 |
|---|---:|---|
| 좌측 흑백 카메라 | 예 | 1280×560, 약 10 Hz |
| 우측 흑백 카메라 | 예 | 1280×560, 약 10 Hz |
| Xsens IMU | 예 | 가속도계 + 자이로, 약 100 Hz |
| LiDAR | 아니요 | 사용하지 않음 |
| GPS/FOG/휠 엔코더 | 아니요 | 추정이나 맵 생성에 사용하지 않음 |

Urban 28 보정 파일에서 사용한 스테레오 베이스라인은 약 `0.475149 m`입니다. 외부 보정은 OpenVINS의 `config/kaist/kalibr_imucam_chain.yaml`을 기준으로 합니다.

## 중요한 좌표계 주의사항

OpenVINS가 출력하는 자세는 `q_GtoI`, 즉 Global → IMU 회전이며 **JPL 쿼터니언** 순서 `[qx, qy, qz, qw]`입니다. Eigen은 Hamilton 규약을 사용하므로 그대로 `Eigen::Quaterniond`에 넣으면 회전 구간이 반대 방향으로 변환됩니다.

이 저장소의 `stereo_cloud.cpp`는 JPL → Hamilton 변환 시 벡터부 부호를 바꾸고, 다음 식으로 카메라 포인트를 전역 좌표로 변환합니다.

```text
p_G = R_ItoG * (R_CtoI * p_C + p_CinI) + p_IinG
```

또한 스테레오 정류 영상에서 얻은 점은 `R_rect^T`를 적용해 원래 카메라 프레임으로 되돌린 후 외부 보정을 적용합니다.

## 1. 의존성 설치

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  libeigen3-dev libopencv-dev libceres-dev \
  libboost-system-dev libboost-filesystem-dev \
  libboost-thread-dev libboost-date-time-dev \
  python3-opencv python3-numpy
```

## 2. OpenVINS 빌드 (ROS 없음)

```bash
cd /root
git clone https://github.com/rpng/open_vins.git
cd open_vins
git checkout 69488123ed9362dd44b6f28e7f4680abbff1442b

cmake -S ov_msckf -B ov_msckf/build \
  -DENABLE_ROS=OFF \
  -DENABLE_ARUCO_TAGS=OFF
cmake --build ov_msckf/build -j"$(nproc)"
```

다른 위치에 OpenVINS를 설치했다면 실행할 때 `OPENVINS_ROOT` 환경변수를 지정하면 됩니다.

## 3. 이 프로젝트 빌드

```bash
cd /root
git clone https://github.com/kangminsu1/openvins-kaist-rosfree.git
cd openvins-kaist-rosfree

cmake -S . -B build \
  -DOPENVINS_ROOT=/root/open_vins \
  -DOPENVINS_BUILD=/root/open_vins/ov_msckf/build
cmake --build build -j"$(nproc)"
```

## 4. 샘플 데이터

시험에 사용한 데이터는 **KAIST Complex Urban Dataset의 Urban 28 (Pangyo)** 입니다.

- [KAIST Complex Urban Dataset](https://sites.google.com/view/complex-urban-dataset)
- [Urban 데이터 다운로드 페이지](https://sites.google.com/view/complex-urban-dataset/download-lidar-stereo)
- [OpenVINS KAIST 데이터 설명](https://docs.openvins.com/gs-datasets.html#gs-data-kaist)

다운로드와 압축 해제 후 최소한 다음 구조가 필요합니다.

```text
urban28-pankyo/
├── calibration/
│   ├── left.yaml
│   └── right.yaml
├── image/
│   ├── stereo_left/<timestamp>.png
│   └── stereo_right/<timestamp>.png
└── sensor_data/
    ├── stereo_stamp.csv
    └── xsens_imu.csv
```

원본 데이터의 저작권과 이용 조건은 KAIST 데이터셋 제공자의 정책을 따릅니다. 데이터 파일은 이 저장소에 포함하지 않습니다.

## 5. 실행

전체 데이터 처리:

```bash
cd /root/openvins-kaist-rosfree
chmod +x run_kaist.sh

./run_kaist.sh \
  /root/datasets/kaist_urban28_archives/urban28-pankyo \
  /root/results/urban28 \
  0
```

인수는 다음과 같습니다.

```text
./run_kaist.sh DATASET_DIR [OUTPUT_DIR] [MAX_FRAMES]
```

- `MAX_FRAMES=0`: 전체 프레임 처리
- 빠른 시험: `MAX_FRAMES=500`

OpenVINS가 `/opt/open_vins`에 있다면:

```bash
OPENVINS_ROOT=/opt/open_vins ./run_kaist.sh /data/urban28-pankyo ./results 500
```

## 개별 실행

궤적만 생성:

```bash
./build/openvins_kaist_rosfree \
  /data/urban28-pankyo \
  /root/open_vins/config/kaist/estimator_config.yaml \
  ./trajectory_openvins.csv \
  0
```

3D 포인트 클라우드 생성:

```bash
./build/kaist_stereo_cloud \
  /data/urban28-pankyo \
  ./trajectory_openvins.csv \
  ./cloud.pcd \
  10 4 15 0
```

마지막 네 값은 각각 프레임 간격, 픽셀 간격, 최대 깊이(m), 최대 pose 수입니다. Urban 28과 같은 야외 스테레오 데이터에서 먼 거리는 깊이 오차가 급격히 커지므로 기본 최대 깊이를 15m로 사용합니다.

## 출력 형식

`trajectory_openvins.csv`:

```text
index,timestamp,x,y,z,qx,qy,qz,qw
```

- 위치: `p_IinG` (전역 좌표계에서 본 IMU 위치, meter)
- 자세: `q_GtoI` (Global → IMU, JPL quaternion)

`cloud.pcd`는 `x y z rgb` 필드를 가진 binary PCD입니다. CloudCompare 또는 PCL 도구로 열 수 있습니다.

## 보정 검사

좌우 스테레오 정렬 상태 확인:

```bash
python3 check_stereo_calibration.py /data/urban28-pankyo
```

Urban 28 시험에서는 수직 오차 중앙값 `0.0 px`, 1픽셀 이내 매칭 약 `91%`를 확인했습니다.

## 제한 사항

- 루프 클로징이 없어 장거리 주행에서 VIO 누적 오차가 발생합니다.
- 움직이는 차량과 보행자는 StereoSGBM 포인트에 노이즈를 만들 수 있습니다.
- 3D 생성기의 카메라–IMU 외부 보정값은 현재 KAIST Urban 28 설정에 맞춰져 있습니다.
- `cloud.pcd`는 조밀한 포인트 클라우드라 수백 MB가 될 수 있습니다.
- 이 코드는 연구 및 재현 실험용입니다. 차량 제어 또는 안전 필수 시스템에 바로 사용하면 안 됩니다.

## 관련 프로젝트와 인용

- [OpenVINS](https://github.com/rpng/open_vins)
- [KAIST Complex Urban Dataset](https://sites.google.com/view/complex-urban-dataset)

연구 결과에 사용할 경우 OpenVINS 및 KAIST Complex Urban Dataset의 원 논문을 함께 인용하세요.

