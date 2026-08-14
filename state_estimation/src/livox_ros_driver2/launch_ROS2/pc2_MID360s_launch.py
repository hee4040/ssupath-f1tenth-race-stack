import os
from launch import LaunchDescription
from launch_ros.actions import Node

################### user configure parameters for ros2 start ###################
# rviz_MID360s_launch.py에서 rviz2만 뺀 주행용 launch.
# msg_MID360s_launch.py는 xfer_format=1(CustomMsg)이라 scanmatcher(PointCloud2 구독)와 호환 안 됨.
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
# ★ 10.0 에서 올리지 말 것 — 20.0 을 실주행에서 시도했다가 측위가 흔들리며 깨졌다 (2026-07-23 검증됨)
#
# 올리고 싶어지는 이유: end-to-end 지연 142ms 중 sweep 누적이 99.8ms(71%)이고
# align 은 중앙 17ms 뿐이라, 지연을 줄이는 유일하게 의미 있는 레버가 이 값이다.
#
# 그런데 안 되는 이유 — 원거리점 손실 (0723_1646 bag 실측):
# MID360 비반복 패턴은 sweep 전반부에 근거리, 후반부에 원거리를 훑는다.
#   dt 0~50ms  : 점 ~10000개, 최대거리 8~9m, 10m이상 점 0개
#   dt 50~100ms: 점 ~10000개, 최대거리 29m,  10m이상 점 약 1000개
# (연속 8프레임에서 동일한 패턴 확인. 점 개수 자체는 10ms 구간마다 ~2000개로 균일하다.)
# 20Hz 로 쪼개면 '근거리만 있는 프레임'과 '원거리 있는 프레임'이 번갈아 나온다.
# 원거리점은 지렛대가 길어 yaw 를 가장 강하게 구속하므로, 절반의 프레임에서 그걸 잃으면
# yaw 가 구속되지 않아 측위가 흔들린다. 서킷이 작아 근거리로 충분할 것이라 기대했으나
# 실주행 결과 그렇지 않았다.
#
# 이 값을 올려서 지연을 줄이려는 시도는 하지 말 것. 지연은 EKF 가 보상하도록 설계돼 있고
# (ekf_localizer 의 extend_state_step), 실측에서 지연 게이트 폐기는 0건이었으며
# 제어기는 EKF 출력을 100Hz 로 받는다. 즉 이 지연은 제어 성능을 실제로 깎고 있지 않다.
#
# 굳이 갱신율을 올리려면 sweep 을 잘라 나누는 방식(publish_freq)이 아니라,
# 100ms 창을 50ms 마다 '겹쳐서' 내보내는 슬라이딩 윈도우여야 한다(드라이버 옵션에 없음).
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_config_path, 'MID360s_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code},
    {"3d_loc_debug": True},
]


def generate_launch_description():
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
        )

    return LaunchDescription([
        livox_driver,
    ])
