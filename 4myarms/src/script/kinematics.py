#!/usr/bin/env python3
import rospy
import PyKDL as kdl
from kdl_parser_py.urdf import treeFromUrdfModel
from urdf_parser_py.urdf import URDF
import os
import tf
import sympy as sp
import numpy as np

# sudo apt install ros-noetic-kdl-parser-py

def create_kdl_chain(urdf_path):
    robot_model = URDF.from_xml_file(urdf_path)
    ok, tree = treeFromUrdfModel(robot_model)
    # (ok, tree) = treeFromParam('/robot_description')
    if not ok:
        rospy.logerr("无法从URDF创建KDL树")
        return None
    # for child_link in tree.getChildren("base_link"):
    #     chain = tree.getChain(root_link, child_link)
    #     print_chain(chain, prefix=f"{root_link} -> {child_link}: ")

    # for i in range(chain.getNrOfSegments()):
    #     segment = chain.getSegment(i)
    #     joint = segment.getJoint()
    #     print(f"{prefix}Segment: {segment.getName()}, Joint: {joint.getName()}, Type: {joint.getType()}")

    base_link = "base_link"
    end_link = "link6"
    chain = tree.getChain(base_link, end_link)
    if chain is None:
        print("failed")
    return chain

def symbolic_transform(chain):
    # 计算变换矩阵（好像有问题）
    joint_symbols = [sp.Symbol(f'q{i}') for i in range(chain.getNrOfJoints())]
    T = sp.eye(4)  # 初始化为单位矩阵
    print(chain.getNrOfSegments())
    for i in range(chain.getNrOfSegments()):
        segment = chain.getSegment(i)
        joint = segment.getJoint()  # 关节信息（Joint）：描述该段的关节类型（旋转、平移等）和其运动自由度
        frame = segment.getFrameToTip()  # 固定变换（Frame to Tip）：描述该段相对于上一段的静态坐标变换

        # 提取静态变换矩阵
        static_transform = frame_to_symbolic(frame)
        # print(static_transform)
        # print(segment.getName())
        # 提取关节变换矩阵

        print(joint.JointAxis())

        if list(joint.JointAxis()) == [0,0,1]:
            print("RotZ")
            theta = joint_symbols.pop(0)
            joint_transform = sp.Matrix([
                [sp.cos(theta), -sp.sin(theta), 0, 0],
                [sp.sin(theta), sp.cos(theta),  0, 0],
                [0,             0,              1, 0],
                [0,             0,              0, 1]
            ])
        elif list(joint.JointAxis()) == [0,1,0]:
            print("RotY")
            d = joint_symbols.pop(0)
            joint_transform = sp.Matrix([
                [sp.cos(theta), 0, sp.sin(theta), 0],
                [0,             1, 0,             0],
                [-sp.sin(theta),0, sp.cos(theta), 0],
                [0,             0, 0,             1]
            ])
        elif list(joint.JointAxis()) == [1,0,0]:
            print("RotX")
            joint_transform = sp.Matrix([
                [1, 0,             0,              0],
                [0, sp.cos(theta), -sp.sin(theta), 0],
                [0, sp.sin(theta), sp.cos(theta),  0],
                [0, 0,             0,              1]
            ])  # 其他类型的关节

        # 累乘当前段的变换
        T = T * joint_transform * static_transform

    return T

def frame_to_symbolic(frame):
    """将 PyKDL Frame 转换为符号形式的 4x4 齐次变换矩阵"""
    translation = frame.p
    rotation = frame.M
    T = sp.eye(4)
    # 平移部分
    T[0, 3] = translation[0]
    T[1, 3] = translation[1]
    T[2, 3] = translation[2]


    # 旋转部分
    for i in range(3):
        for j in range(3):
            T[i, j] = rotation[i, j]
    
    return T


def forward_kinematics(chain, joint_angles):
    # 创建求解器
    fk_solver = kdl.ChainFkSolverPos_recursive(chain)
    # 设置关节角度
    joint_num = chain.getNrOfJoints()
    # print(joint_num)

    # for i in range(chain.getNrOfSegments()):
    #     segment = chain.getSegment(i)
    #     print(f"段 {i}: {segment.getName()}")
    
    joint_array = kdl.JntArray(chain.getNrOfJoints())
    for i, angle in enumerate(joint_angles):
        joint_array[i] = angle

    
    # 计算前向运动学
    end_frame = kdl.Frame()
    fk_solver.JntToCart(joint_array, end_frame)
    return end_frame


def inverse_kinematics(chain, target_frame):
    # 创建IK求解器
    ik_solver = kdl.ChainIkSolverPos_LMA(chain)
    joint_array = kdl.JntArray(chain.getNrOfJoints())
    result_angles = kdl.JntArray(chain.getNrOfJoints())
    if ik_solver.CartToJnt(joint_array, target_frame, result_angles) >= 0:
        return [result_angles[i] for i in range(result_angles.rows())]
    else:
        rospy.logerr("逆向运动学求解失败")
        return None

def quaternion_to_rotation_matrix(q):
    """
    将四元数 (qx, qy, qz, qw) 转换为 3x3 旋转矩阵
    q = (qx, qy, qz, qw)
    """
    qx, qy, qz, qw = q
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
    ])
    return R

def create_homogeneous_transform(translation, rotation_quaternion):
    """
    根据平移向量和旋转四元数生成齐次变换矩阵
    translation: 平移向量 (x, y, z)
    rotation_quaternion: 旋转四元数 (qx, qy, qz, qw)
    """
    # 获取旋转矩阵
    R = quaternion_to_rotation_matrix(rotation_quaternion)
    
    # 创建齐次变换矩阵
    T = np.eye(4)  # 初始化一个 4x4 单位矩阵
    T[:3, :3] = R  # 将旋转矩阵填入左上角 3x3 部分
    T[:3, 3] = translation  # 将平移向量填入前三列的最后一列
    
    return T



if __name__ == '__main__':
    urdf_path = '/home/znfs/robot_scientist/follow1/src/arm_control/models/fl.urdf'
    # with open(urdf_path, "r") as file:
    #     urdf_content = file.read()
    # rospy.set_param("/robot_description", urdf_content)
    # robot_param = URDF.from_parameter_server()
    # print(robot)

    chain = create_kdl_chain(urdf_path)

    #symbolic_transform(chain)

    # 正向运动学示例
    # joint_angles = [0.83867359,1.82440662,1.34832573,0.8871212,0.11272621,-0.02651215] 
    # clawArm2
    # joint_angles = [0.4938201611019663, 2.0525142272245818, 0.6914252185551759, 1.3173726160790065, -0.09899364626622334, -0.13218755403944227]
    # clawArm1
    # joint_angles = [ 0.83867359 , 1.82440662 , 1.34832573 , 0.8871212 ,  0.11272621 , -0.02651215]
    #clawArm4
    # joint_angles = [0.02727509, 0.01049042, 0.02994537,-0.21992111,0.03261662,0.05321598]
    joint_angles = [0.0616,1.7411,1.0405,0.7005,0.0617,0]
    end_frame = forward_kinematics(chain, joint_angles)
    print("末端位姿:", end_frame)

    # 逆向运动学示例
    target_frame = kdl.Frame()
    # clawArm2
    # target_frame.p = kdl.Vector(0.207755,0.1676824,-0.057405)
    # target_frame.M = kdl.Rotation.RPY(-0.127857,0.0434655,0.5929007)

    # clawArm1
    # target_frame.p = kdl.Vector(0.185414,0.310159,0.175164)
    # target_frame.M = kdl.Rotation.RPY(0.022483,-0.408275,0.715797)
    target_frame.p = kdl.Vector(0.381604,0.0227833,0.245535)
    rot_matrix = np.array([[1, 9.27588e-05, 9.98876e-05], [-9.27601e-05, 1, 1.22605e-05], [-9.98865e-05,-1.22698e-05, 1]])
    target_frame.M = kdl.Rotation(rot_matrix[0, 0], rot_matrix[0, 1], rot_matrix[0, 2],
                                rot_matrix[1, 0], rot_matrix[1, 1], rot_matrix[1, 2],
                                rot_matrix[2, 0], rot_matrix[2, 1], rot_matrix[2, 2])
    ik_result = inverse_kinematics(chain, target_frame)
    if ik_result:
        print("逆向运动学解:", ik_result)

    # 由tf得到齐次变换矩阵
    translation1 = np.array([0.0, 0, 0.0603])
    rotation_quaternion1 = np.array([0.0, 0.0, 0.0, 1.0])
    translation2 = np.array([0.02, 0, 0.0402])
    rotation_quaternion2 = np.array([0.0, 0.7646, 0.0, 0.6444])
    translation3 = np.array([-0.264, 0.0, 0.0])
    rotation_quaternion3 = np.array([0.8676, 0, 0.4971, 0])
    translation4 = np.array([0.245, 0, -0.056])
    rotation_quaternion4 = np.array([0.0, 0.3431, 0.0, 0.9392])
    translation5 = np.array([0.06575, -0.001, -0.0825])
    rotation_quaternion5 = np.array([0.0, 0.0, 0.0308, 0.9995])
    translation6 = np.array([0.02845, 0.0, 0.0825])
    rotation_quaternion6 = np.array([1, 0.0, 0.0, 0])
    T1 = create_homogeneous_transform(translation1, rotation_quaternion1)
    T2 = create_homogeneous_transform(translation2, rotation_quaternion2)
    T3 = create_homogeneous_transform(translation3, rotation_quaternion3)
    T4 = create_homogeneous_transform(translation4, rotation_quaternion4)
    T5 = create_homogeneous_transform(translation5, rotation_quaternion5)
    T6 = create_homogeneous_transform(translation6, rotation_quaternion6)
    T = T1 @ T2 @ T3 @ T4 @ T5 @ T6
    print("齐次变换矩阵:", T)


    print('实际关节角度:', joint_angles)
    print('根据tf得到的末端齐次变换矩阵:',T)
    print('由正运动学求解得到的末端位置:',end_frame.p)
    print('由正运动学求解得到的末端姿态:',end_frame.M)
    print('由末端位姿根据逆运动学求解得到的关节角度:',ik_result)
