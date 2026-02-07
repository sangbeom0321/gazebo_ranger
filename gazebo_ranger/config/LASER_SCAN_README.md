# PointCloud to LaserScan 변환 노드 설정 가이드

## 개요
`laser_scan` 노드는 3D 포인트클라우드를 2D LaserScan으로 변환합니다.
IMU 데이터를 사용하여 로봇의 기울기를 보정하고, Z축 클리핑과 노이즈 제거를 수행합니다.

## 설정 가능한 파라미터

### 토픽 설정
| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `pcd_topic` | `/points` | 입력 포인트클라우드 토픽 |
| `scan_topic` | `/scan` | 출력 LaserScan 토픽 |
| `imu_topic` | `/imu` | IMU 토픽 (기울기 보정용) |

### Z축 클리핑 범위 (미터)
로봇이 기울어질 때 동적으로 조정됩니다.

| 파라미터 | 기본값 | 범위 | 설명 |
|---------|--------|------|------|
| `clipping_minz` | `-0.2` | `-2.0 ~ 0.0` | 최소 Z값 (미터) |
| `clipping_maxz` | `0.4` | `0.0 ~ 2.0` | 최대 Z값 (미터) |

**동적 조정 공식**: 
- `adjusted_minz = clipping_minz + (1.0 - cos(roll)*cos(pitch)) * 0.6`
- `adjusted_maxz = clipping_maxz + (1.0 - cos(roll)*cos(pitch)) * 0.6`

### LaserScan 설정

#### 각도 설정 (라디안)
| 파라미터 | 기본값 | 범위 | 설명 |
|---------|--------|------|------|
| `angle_min` | `-π` (-180°) | `-π ~ 0` | 최소 스캔 각도 |
| `angle_max` | `π` (180°) | `0 ~ π` | 최대 스캔 각도 |
| `angle_increment` | `π/180` (1°) | `0.001 ~ 0.1` | 각도 간격 |

**권장값**:
- 고해상도: `0.0087` (0.5도)
- 표준: `0.0175` (1도) ← 기본값
- 저해상도: `0.0349` (2도)

#### 거리 설정 (미터)
| 파라미터 | 기본값 | 범위 | 설명 |
|---------|--------|------|------|
| `range_min` | `0.5` | `0.1 ~ 5.0` | 최소 측정 거리 |
| `range_max` | `200.0` | `10.0 ~ 500.0` | 최대 측정 거리 |

**권장값**:
- 실내: `range_max: 30.0`
- 실외: `range_max: 200.0` ← 기본값
- 장거리: `range_max: 500.0`

#### 프레임 ID
| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `scan_frame_id` | `laser_data_frame` | LaserScan 메시지의 frame_id |

### 포인트클라우드 필터링 설정

#### Radius Outlier Removal
노이즈 포인트를 제거합니다.

| 파라미터 | 기본값 | 범위 | 설명 |
|---------|--------|------|------|
| `radius_search` | `0.3` | `0.1 ~ 1.0` | 검색 반경 (미터) |
| `min_neighbors_in_radius` | `3` | `1 ~ 10` | 최소 이웃 포인트 개수 |

**권장값**:
- 조밀한 포인트클라우드: `radius_search: 0.2`, `min_neighbors: 2`
- 일반: `radius_search: 0.3`, `min_neighbors: 3` ← 기본값
- 희소한 포인트클라우드: `radius_search: 0.5`, `min_neighbors: 5`

## 사용 예시

### Launch 파일에서 파라미터 설정
```python
laser_scan_node = Node(
    package='gazebo_ranger',
    executable='laser_scan',
    name='laser_scan',
    parameters=[
        {'use_sim_time': True},
        {'pcd_topic': '/points'},
        {'scan_topic': '/scan'},
        {'imu_topic': '/imu'},
        {'range_max': 30.0},  # 실내용으로 변경
        {'angle_increment': 0.0087}  # 0.5도 간격
    ],
    output='screen'
)
```

### YAML 파일 사용
`config/laser_scan_params.yaml` 파일을 수정하거나 launch 파일에서 참조:
```python
laser_scan_config = join(pkg_share, 'config', 'laser_scan_params.yaml')
laser_scan_node = Node(
    package='gazebo_ranger',
    executable='laser_scan',
    parameters=[laser_scan_config]
)
```

## 동작 원리

1. **포인트클라우드 수신**: `/points` 토픽에서 PointCloud2 메시지 수신
2. **IMU 기울기 보정**: IMU 데이터로 roll/pitch 계산하여 포인트클라우드 변환
3. **Z축 클리핑**: 기울기에 따라 동적으로 조정된 Z 범위로 필터링
4. **노이즈 제거**: Radius Outlier Removal로 이상치 제거
5. **LaserScan 변환**: 3D 포인트를 2D 각도-거리로 변환
6. **발행**: `/scan` 토픽으로 LaserScan 메시지 발행

## 주의사항

- IMU 데이터가 없으면 포인트클라우드 처리가 중단됩니다
- `/clock` 토픽이 있으면 시뮬레이션 시간을 사용합니다
- 같은 각도에 여러 포인트가 있으면 가장 가까운 거리를 사용합니다
- Yaw 회전은 무시됩니다 (Roll, Pitch만 사용)
