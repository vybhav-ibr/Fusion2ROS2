# -*- coding: utf-8 -*-

import adsk, os
from xml.etree.ElementTree import Element, SubElement
from . import Link, Joint, Sensors_GZC, Sensors_GZS, Sensor_GZC_Parameter,Sensor_GZS_Parameter, launch_templates
from ..utils import utils

def write_link_urdf(joints_dict, repo, links_xyz_dict, file_name, inertial_dict):
    """
    Write links information into urdf "repo/file_name"


    Parameters
    ----------
    joints_dict: dict
        information of the each joint
    repo: str
        the name of the repository to save the xml file
    links_xyz_dict: vacant dict
        xyz information of the each link
    file_name: str
        urdf full path
    inertial_dict:
        information of the each inertial

    Note
    ----------
    In this function, links_xyz_dict is set for write_joint_tran_urdf.
    The origin of the coordinate of center_of_mass is the coordinate of the link
    """
    with open(file_name, mode='a') as f:
        # for base_link
        center_of_mass = inertial_dict['base_link']['center_of_mass']
        link = Link.Link(name='base_link', xyz=[0,0,0],
            center_of_mass=center_of_mass, repo=repo,
            mass=inertial_dict['base_link']['mass'],
            inertia_tensor=inertial_dict['base_link']['inertia'])
        links_xyz_dict[link.name] = link.xyz
        link.make_link_xml()
        f.write(link.link_xml)
        f.write('\n')

        # others
        for joint in joints_dict:
            name = joints_dict[joint]['child']
            center_of_mass = \
                [ i-j for i, j in zip(inertial_dict[name]['center_of_mass'], joints_dict[joint]['xyz'])]
            link = Link.Link(name=name, xyz=joints_dict[joint]['xyz'],\
                center_of_mass=center_of_mass,\
                repo=repo, mass=inertial_dict[name]['mass'],\
                inertia_tensor=inertial_dict[name]['inertia'])
            links_xyz_dict[link.name] = link.xyz
            link.make_link_xml()
            f.write(link.link_xml)
            f.write('\n')


def write_joint_urdf(joints_dict, repo, links_xyz_dict, file_name):
    """
    Write joints and transmission information into urdf "repo/file_name"


    Parameters
    ----------
    joints_dict: dict
        information of the each joint
    repo: str
        the name of the repository to save the xml file
    links_xyz_dict: dict
        xyz information of the each link
    file_name: str
        urdf full path
    """

    with open(file_name, mode='a') as f:
        for j in joints_dict:
            parent = joints_dict[j]['parent']
            child = joints_dict[j]['child']
            joint_type = joints_dict[j]['type']
            upper_limit = joints_dict[j]['upper_limit']
            lower_limit = joints_dict[j]['lower_limit']
            try:
                xyz = [round(p-c, 6) for p, c in \
                    zip(links_xyz_dict[parent], links_xyz_dict[child])]  # xyz = parent - child
            except KeyError as ke:
                app = adsk.core.Application.get()
                ui = app.userInterface
                ui.messageBox("There seems to be an error with the connection between\n\n%s\nand\n%s\n\nCheck \
whether the connections\nparent=component2=%s\nchild=component1=%s\nare correct or if you need \
to swap component1<=>component2"
                % (parent, child, parent, child), "Error!")
                quit()

            joint = Joint.Joint(name=j, joint_type = joint_type, xyz=xyz, \
            axis=joints_dict[j]['axis'], parent=parent, child=child, \
            upper_limit=upper_limit, lower_limit=lower_limit)
            joint.make_joint_xml()
            joint.make_control_xml()
            f.write(joint.joint_xml)
            f.write('\n')
            if "DCAM" in child:
                f.write(f'<link name="{utils.extract_prefix(child)}_optical"/>\n')
                f.write(f'<joint name="{utils.extract_prefix(child)}_optical_joint" type="fixed">\n')
                f.write(f'    <parent link="{child}"/>\n')
                f.write(f'    <child link="{utils.extract_prefix(child)}_optical"/>\n')
                f.write(f'    <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>\n')
                f.write(f'</joint>\n')
                f.write('\n')


def write_robot_endtag(file_name):
    with open(file_name, mode='a') as f:
        f.write('</robot>\n')



def write_urdf(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir):
    try: os.mkdir(save_dir + '/urdf')
    except: pass
    print(save_dir)
    file_name = save_dir + '/urdf/' + robot_name + '.xacro'  # the name of urdf file
    repo = package_name + '/meshes/'  # the repository of binary stl files
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'.format(robot_name))
        f.write('\n')
        f.write('<xacro:include filename="$(find {})/urdf/materials.xacro" />'.format(package_name))
        f.write('\n')
        f.write('<xacro:include filename="$(find {})/urdf/{}.xacro" />'.format(package_name, 'sensors'))
        f.write('\n')
        f.write('<xacro:include filename="$(find {})/urdf/{}.gazebo" />'.format(package_name, robot_name))
        f.write('\n')

    write_link_urdf(joints_dict, repo, links_xyz_dict, file_name, inertial_dict)
    write_joint_urdf(joints_dict, repo, links_xyz_dict, file_name)
    write_robot_endtag(file_name)

def write_materials_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir):
    try: os.mkdir(save_dir + '/urdf')
    except: pass

    file_name = save_dir + '/urdf/materials.xacro'  # the name of urdf file
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n'.format(robot_name))
        f.write('\n')
        f.write('<material name="silver">\n')
        f.write('  <color rgba="0.700 0.700 0.700 1.000"/>\n')
        f.write('</material>\n')
        f.write('\n')
        f.write('</robot>\n')

def write_transmissions_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir):
    """
    Write joints and transmission information into urdf "repo/file_name"


    Parameters
    ----------
    joints_dict: dict
        information of the each joint
    repo: str
        the name of the repository to save the xml file
    links_xyz_dict: dict
        xyz information of the each link
    file_name: str
        urdf full path
    """

    file_name = save_dir + '/urdf/{}.trans'.format(robot_name)  # the name of urdf file
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n'.format(robot_name))
        f.write('\n')

        for j in joints_dict:
            parent = joints_dict[j]['parent']
            child = joints_dict[j]['child']
            joint_type = joints_dict[j]['type']
            upper_limit = joints_dict[j]['upper_limit']
            lower_limit = joints_dict[j]['lower_limit']
            try:
                xyz = [round(p-c, 6) for p, c in \
                    zip(links_xyz_dict[parent], links_xyz_dict[child])]  # xyz = parent - child
            except KeyError as ke:
                app = adsk.core.Application.get()
                ui = app.userInterface
                ui.messageBox("There seems to be an error with the connection between\n\n%s\nand\n%s\n\nCheck \
whether the connections\nparent=component2=%s\nchild=component1=%s\nare correct or if you need \
to swap component1<=>component2"
                % (parent, child, parent, child), "Error!")
                quit()

            joint = Joint.Joint(name=j, joint_type = joint_type, xyz=xyz, \
            axis=joints_dict[j]['axis'], parent=parent, child=child, \
            upper_limit=upper_limit, lower_limit=lower_limit)
            if joint_type != 'fixed':
                joint.make_transmission_xml()
                f.write(joint.tran_xml)
                f.write('\n')

        f.write('</robot>\n')

def write_gazebo_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir,GZB_Version):
    try: os.mkdir(save_dir + '/urdf')
    except: pass

    file_name = save_dir + '/urdf/' + robot_name + '.gazebo'  # the name of urdf file
     #repo = package_name + '/' + robot_name + '/bin_stl/'  # the repository of binary stl files
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n'.format(robot_name))
        f.write('\n')
        f.write('<xacro:property name="body_color" value="Gazebo/Silver" />\n')
        f.write('\n')

        gazebo = Element('gazebo')
        plugin = SubElement(gazebo, 'plugin')
        plugin.attrib = {'name':'control', 'filename':'libgazebo_ros_control.so'}
        gazebo_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        # f.write(gazebo_xml)

        # for base_link
        f.write('<gazebo reference="base_link">\n')
        if GZB_Version == 'GZC': f.write('  <material>${body_color}</material>\n')
        f.write('  <mu1>0.2</mu1>\n')
        f.write('  <mu2>0.2</mu2>\n')
        f.write('  <self_collide>true</self_collide>\n')
        f.write('  <gravity>true</gravity>\n')
        f.write('</gazebo>\n')
        f.write('\n')

        # others
        for joint in joints_dict:
            name = joints_dict[joint]['child']
            f.write('<gazebo reference="{}">\n'.format(name))
            if GZB_Version == 'GZC': f.write('  <material>${body_color}</material>\n')
            f.write('  <mu1>0.2</mu1>\n')
            f.write('  <mu2>0.2</mu2>\n')
            f.write('  <self_collide>true</self_collide>\n')
            f.write('</gazebo>\n')
            f.write('\n')
        # f.write('<gazebo>\n')
        if GZB_Version == 'GZC':
            f.write('<ros2_control name="GazeboSystem" type="system">\n')
        else:
            f.write('<ros2_control name="GazeboSimSystem" type="system">\n')
        f.write('<hardware>\n')
        if GZB_Version == 'GZC':
            f.write('   <plugin>gazebo_ros2_control/GazeboSystem</plugin>\n')
        else:
            f.write('   <plugin>gz_ros2_control/GazeboSimSystem</plugin>\n')
        f.write('</hardware>\n')
        for j in joints_dict:
            parent = joints_dict[j]['parent']
            child = joints_dict[j]['child']
            joint_type = joints_dict[j]['type']
            upper_limit = joints_dict[j]['upper_limit']
            lower_limit = joints_dict[j]['lower_limit']
            if joint_type.lower() != 'fixed':
                if abs(upper_limit)>0.0 or  abs(lower_limit)>0.0:
                    joint = Joint.Joint(name=j, joint_type = joint_type, xyz=[0.0,0.0,0.0], \
                    axis=joints_dict[j]['axis'], parent=parent, child=child, \
                    upper_limit=upper_limit, lower_limit=lower_limit)
                    joint.make_control_xml()
                    f.write(joint.control_xml)
                    f.write('\n')
                else:
                    f.write(f'<joint name="{j}">\n')
                    f.write('   <state_interface name="position"/>\n')
                    f.write('   <state_interface name="velocity"/>\n')
                    f.write('</joint>\n')
                    f.write('\n')
        f.write('</ros2_control>\n')
        f.write('<gazebo>\n')
        if GZB_Version =='GZC':
            f.write('<plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">\n')
            f.write('   <parameters>$(find {})/config/ros2_controller.yaml</parameters>\n'.format(package_name))
            f.write('</plugin>\n')
        else:
            f.write('<plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">\n')
            f.write('   <parameters>$(find {})/config/ros2_controller.yaml</parameters>\n'.format(package_name))
            f.write('</plugin>\n')
            f.write('<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">\n')
            f.write('   <render_engine>ogre2</render_engine>\n')
            f.write('</plugin>\n')
            f.write('<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>\n')
        f.write('</gazebo>\n')
        f.write('</robot>\n')
        
def write_sensors_xacro(sensors_dict, package_name,robot_name, save_dir,GZB_Version):
    try: os.mkdir(save_dir + '/urdf')
    except: pass

    file_name = save_dir + '/urdf/' + 'sensors' + '.xacro'  # the name of urdf file
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n'.format(robot_name))
        f.write('\n')
        f.write('<xacro:property name="yaml_file" value="$(find {})/config/sensor_parameters.yaml" />\n'.format(package_name))
        f.write('<xacro:property name="params_dict" value="${xacro.load_yaml(yaml_file)}" />\n')
        f.write('''<xacro:property name="L2D_params" value="${params_dict['L2D']}" />\n''')
        f.write('''<xacro:property name="L3D_params" value="${params_dict['L3D']}" />\n''')
        f.write('''<xacro:property name="DCAM_params" value="${params_dict['DCAM']}" />\n''')
        f.write('''<xacro:property name="CAM_params" value="${params_dict['CAM']}" />\n''')
        f.write('''<xacro:property name="IMU_params" value="${params_dict['IMU']}" />\n''')
        f.write('''<xacro:property name="GPS_params" value="${params_dict['GPS']}" />\n''')
        f.write('''<xacro:property name="P3D_params" value="${params_dict['P3D']}" />\n''')
        if GZB_Version=='GZC':
            for key in sensors_dict:
                if sensors_dict[key]=='L2D':
                    f.write(Sensors_GZC.Sensors_GZC.make_L2D(key))
                    f.write('\n')
                elif sensors_dict[key]=='L3D':
                    f.write(Sensors_GZC.Sensors_GZC.make_L3D(key))
                    f.write('\n')
                elif sensors_dict[key]=='DCAM':
                    f.write(Sensors_GZC.Sensors_GZC.make_DCAM(key))
                    f.write('\n')
                elif sensors_dict[key]=='CAM':
                    f.write(Sensors_GZC.Sensors_GZC.make_CAM(key))
                    f.write('\n')
                elif sensors_dict[key]=='IMU':
                    f.write(Sensors_GZC.Sensors_GZC.make_IMU(key))
                    f.write('\n')
                elif sensors_dict[key]=='GPS':
                    f.write(Sensors_GZC.Sensors_GZC.make_GPS(key))
                    f.write('\n')
                elif sensors_dict[key]=='P3D':
                    f.write(Sensors_GZC.Sensors_GZC.make_P3D(key))
                    f.write('\n')
        else:
            for key in sensors_dict:
                if sensors_dict[key]=='L2D':
                    f.write(Sensors_GZS.Sensors_GZS.make_L2D(key))
                    f.write('\n')
                elif sensors_dict[key]=='L3D':
                    f.write(Sensors_GZS.Sensors_GZS.make_L3D(key))
                    f.write('\n')
                elif sensors_dict[key]=='DCAM':
                    f.write(Sensors_GZS.Sensors_GZS.make_DCAM(key))
                    f.write('\n')
                elif sensors_dict[key]=='CAM':
                    f.write(Sensors_GZS.Sensors_GZS.make_CAM(key))
                    f.write('\n')
                elif sensors_dict[key]=='IMU':
                    f.write(Sensors_GZS.Sensors_GZS.make_IMU(key))
                    f.write('\n')
                elif sensors_dict[key]=='GPS':
                    f.write(Sensors_GZS.Sensors_GZS.make_GPS(key))
                    f.write('\n')
                elif sensors_dict[key]=='P3D':
                    f.write(Sensors_GZS.Sensors_GZS.make_ODOM(key))
                    f.write('\n')

        f.write('</robot>\n')
        
def write_configs(joints_dict,sensors_dict,save_dir,GZB_Version):
    try: os.mkdir(save_dir + '/config')
    except: pass
    actuated_joints=[]
    for j in joints_dict:
        if abs(joints_dict[j]['lower_limit'])>0.0 or  abs(joints_dict[j]['upper_limit'])>0.0:
            actuated_joints.append(j)
    cntrl_yaml_file_name = save_dir + '/config/' + 'ros2_controller' + '.yaml'
    config_data = {
        'controller_manager': {
            'ros__parameters': {
                'update_rate': 1000,  # Hz
            }
        },
        'joint_trajectory_controller': {
            'ros__parameters': {
                'type': 'joint_trajectory_controller/JointTrajectoryController',
                'joints': actuated_joints,
                'command_interfaces': ['position','velocity'],
                'state_interfaces': ['position', 'velocity'],
                'state_publish_rate': 200.0,  # Defaults to 50
                'action_monitor_rate': 20.0,  # Defaults to 20
                'allow_partial_joints_goal': True,  # Defaults to false
                'deduce_states_from_derivatives': True,
                'open_loop_control': False,
                'allow_integration_in_goal_trajectories': True,
                'constraints': {
                    'stopped_velocity_tolerance': 0.01,  # Defaults to 0.01
                    'goal_time': 0.0  # Defaults to 0.0 (start immediately)
                }
            }
        },
        'joint_state_broadcaster': {
            'ros__parameters': {
                'state_publish_rate': 500.0,  # Defaults to 50
                'type': 'joint_state_broadcaster/JointStateBroadcaster'
            }
        }
    }
    if GZB_Version =='GZC':
        Sensor_GZC_Parameter.Sensor_GZC_Parameter.write_yaml(config_data,cntrl_yaml_file_name)
    else:
        Sensor_GZC_Parameter.Sensor_GZC_Parameter.write_yaml(config_data,cntrl_yaml_file_name)
    
    sensor_parameter_file_name=save_dir + '/config/' + 'sensor_parameters' + '.yaml'
    sensor_parameter_dict={}
    sensor_parameter_dict.setdefault("L2D", {})
    sensor_parameter_dict.setdefault("L3D", {})
    sensor_parameter_dict.setdefault("CAM", {})
    sensor_parameter_dict.setdefault("DCAM", {})
    sensor_parameter_dict.setdefault("IMU", {})
    sensor_parameter_dict.setdefault("GPS", {})
    sensor_parameter_dict.setdefault("P3D", {})
    if GZB_Version=='GZC':
        for key in sensors_dict:
            if sensors_dict[key]=='L2D':
                sensor_parameter_dict['L2D'][f'{utils.extract_prefix(key)}_L2D']=Sensor_GZC_Parameter.Sensor_GZC_Parameter.make_L2D(key,'_2DL')
            elif sensors_dict[key]=='L3D':
                sensor_parameter_dict['L3D'][f'{utils.extract_prefix(key)}_L3D']=Sensor_GZC_Parameter.Sensor_GZC_Parameter.make_L3D(key,'_3DL')
            elif sensors_dict[key]=='DCAM':
                sensor_parameter_dict['DCAM'][f'{utils.extract_prefix(key)}_DCAM']=Sensor_GZC_Parameter.Sensor_GZC_Parameter.make_DCAM(key,'_DCAM')
            elif sensors_dict[key]=='CAM':
                sensor_parameter_dict['CAM'][f'{utils.extract_prefix(key)}_CAM']=Sensor_GZC_Parameter.Sensor_GZC_Parameter.make_CAM(key,'_CAM')
            elif sensors_dict[key]=='IMU':
                sensor_parameter_dict['IMU'][f'{utils.extract_prefix(key)}_IMU']=Sensor_GZC_Parameter.Sensor_GZC_Parameter.make_IMU(key,'_IMU')
            elif sensors_dict[key]=='GPS':
                sensor_parameter_dict['GPS'][f'{utils.extract_prefix(key)}_GPS']=Sensor_GZC_Parameter.Sensor_GZC_Parameter.make_GPS(key,'_GPS')
            elif sensors_dict[key]=='P3D':
                sensor_parameter_dict['P3D'][f'{utils.extract_prefix(key)}_P3D']=Sensor_GZC_Parameter.Sensor_GZC_Parameter.make_P3D(key,'_P3D')
        
        Sensor_GZC_Parameter.Sensor_GZC_Parameter.write_yaml(sensor_parameter_dict,sensor_parameter_file_name)
    else:
        for key in sensors_dict:
            if sensors_dict[key]=='L2D':
                sensor_parameter_dict['L2D'][f'{utils.extract_prefix(key)}_L2D']=Sensor_GZS_Parameter.Sensor_GZS_Parameter.make_L2D(key,'_2DL')
            elif sensors_dict[key]=='L3D':
                sensor_parameter_dict['L3D'][f'{utils.extract_prefix(key)}_L3D']=Sensor_GZS_Parameter.Sensor_GZS_Parameter.make_L3D(key,'_3DL')
            elif sensors_dict[key]=='DCAM':
                sensor_parameter_dict['DCAM'][f'{utils.extract_prefix(key)}_DCAM']=Sensor_GZS_Parameter.Sensor_GZS_Parameter.make_DCAM(key,'_DCAM')
            elif sensors_dict[key]=='CAM':
                sensor_parameter_dict['CAM'][f'{utils.extract_prefix(key)}_CAM']=Sensor_GZS_Parameter.Sensor_GZS_Parameter.make_CAM(key,'_CAM')
            elif sensors_dict[key]=='IMU':
                sensor_parameter_dict['IMU'][f'{utils.extract_prefix(key)}_IMU']=Sensor_GZS_Parameter.Sensor_GZS_Parameter.make_IMU(key,'_IMU')
            elif sensors_dict[key]=='GPS':
                sensor_parameter_dict['GPS'][f'{utils.extract_prefix(key)}_GPS']=Sensor_GZS_Parameter.Sensor_GZS_Parameter.make_GPS(key,'_GPS')
            elif sensors_dict[key]=='P3D':
                sensor_parameter_dict['P3D'][f'{utils.extract_prefix(key)}_L2D']=Sensor_GZS_Parameter.Sensor_GZS_Parameter.make_P3D(key,'_P3D')
        
        Sensor_GZS_Parameter.Sensor_GZS_Parameter.write_yaml(sensor_parameter_dict,sensor_parameter_file_name)

def write_display_launch(package_name, robot_name, save_dir):
    """
    write display launch file "save_dir/launch/display.launch"


    Parameter
    ---------
    robot_name: str
    name of the robot
    save_dir: str
    path of the repository to save
    """
    try: os.mkdir(save_dir + '/launch')
    except: pass

    file_text = launch_templates.get_display_launch_text(package_name, robot_name)

    file_name = os.path.join(save_dir, 'launch', 'display.launch.py')
    with open(file_name, mode='w') as f:
        f.write(file_text)

def write_gazebo_launch(package_name, robot_name, save_dir, GZB_Version,sensors_dict):
    """
    write gazebo launch file "save_dir/launch/gazebo.launch"


    Parameter
    ---------
    robot_name: str
        name of the robot
    save_dir: str
        path of the repository to save
    """
    ros_gz_bridge_args,ros_gz_bridge_params=utils.make_ros2_gz_bridge(sensors_dict,robot_name)
    try: os.mkdir(save_dir + '/launch')
    except: pass
    if GZB_Version=='GZC':
        file_text = launch_templates.get_gazebo_launch_text_GZC(package_name, robot_name)
    else:
        file_text = launch_templates.get_gazebo_launch_text_GZS(package_name, robot_name,ros_gz_bridge_args,ros_gz_bridge_params)

    file_name = os.path.join(save_dir, 'launch', 'gazebo.launch.py')
    with open(file_name, mode='w') as f:
        f.write(file_text)
