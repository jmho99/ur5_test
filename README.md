# 실행 순서

## 1. UR 불러오기

* 시뮬레이션

    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=yyy.yyy.yyy.yyy initial_joint_controller:=joint_trajectory_controller use_fake_hardware:=true launch_rviz:=false

* 실제

    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=yyy.yyy.yyy.yyy initial_joint_controller:=joint_trajectory_controller launch_rviz:=false

## 2. UR moveit 실행

* 시뮬레이션

    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 robot_ip:=yyy.yyy.yyy.yyy initial_joint_controller:=joint_trajectory_controller use_fake_hardware:=true launch_rviz:=true

* 실제

    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 robot_ip:=yyy.yyy.yyy.yyy initial_joint_controller:=joint_trajectory_controller launch_rviz:=true

## 3. Iksolver 실행

    ros2 run service_ur5ik calc_server

## 4. ur_mover 실행

    ros2 launch ur_mover master_node.launch.py

## 5. camera 실행

    ros2 run topic_camera camera_pub


---

## 영상

유투브 : https://www.youtube.com/@%EC%A0%95%EB%AA%85%ED%98%B8-l1i

---

