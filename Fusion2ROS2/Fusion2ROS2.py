#Author-syuntoku14
#Description-Generate URDF file from Fusion 360

import adsk, adsk.core, adsk.fusion, traceback
import os
import sys
from .utils import utils
from .core import Link, Joint, Write, Sensors_GZC

"""
# length unit is 'cm' and inertial unit is 'kg/cm^2'
# If there is no 'body' in the root component, maybe the corrdinates are wrong.
"""

# joint effort: 100
# joint velocity: 100
# supports "Revolute", "Rigid" and "Slider" joint types

# I'm not sure how prismatic joint acts if there is no limit in fusion model

def run(context):
    ui = None
    success_msg = 'Successfully create URDF file'
    msg = success_msg

    try:
        # --------------------
        # initialize
        app = adsk.core.Application.get()
        ui = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        title = 'Fusion2ROS2'
        if not design:
            ui.messageBox('No active Fusion design', title)
            return

        root = design.rootComponent  # root component
        components = design.allComponents

        # set the names
        robot_name = root.name.split()[0]
        package_name = robot_name + '_description'
        save_dir = utils.file_dialog(ui)
        if save_dir == False:
            ui.messageBox('Fusion2ROS2 was canceled', title)
            return 0

        GZB_Version, success= utils.input_box_dialog(ui)
        if not success or GZB_Version not in ['GZC','GZS']:
            ui.messageBox('Wrong simulator version entered or unable to get Simulator', title)
            return 0
        save_dir = save_dir + '/' + package_name
        try: os.mkdir(save_dir)
        except: pass

        package_dir = os.path.abspath(os.path.dirname(__file__)) + '/package/'

        # --------------------
        # set dictionaries

        # Generate joints_dict. All joints are related to root.
        joints_dict, msg = Joint.make_joints_dict(root, msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0

        # Generate inertial_dict
        inertial_dict, msg = Link.make_inertial_dict(root, msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0
        elif not 'base_link' in inertial_dict:
            msg = 'There is no base_link. Please set base_link and run again.'
            ui.messageBox(msg, title)
            return 0

        links_xyz_dict = {}
        
        sensors_dict, msg=Sensors_GZC.make_sensors_dict(joints_dict, msg)
        if msg!=success_msg:
            ui.messageBox(msg, title)
            return 0

        # --------------------
        # Generate URDF
        Write.write_urdf(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        Write.write_materials_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        # Write.write_transmissions_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        Write.write_gazebo_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir,GZB_Version)
        Write.write_sensors_xacro(sensors_dict, package_name, robot_name, save_dir, GZB_Version)
        Write.write_display_launch(package_name, robot_name, save_dir)
        Write.write_gazebo_launch(package_name, robot_name, save_dir,GZB_Version,sensors_dict)
        Write.write_configs(joints_dict,sensors_dict,save_dir,GZB_Version)

        # copy over package files
        utils.create_package(package_name, save_dir, package_dir)
        utils.update_setup_py(save_dir, package_name, GZB_Version)
        utils.update_setup_cfg(save_dir, package_name)
        utils.update_package_xml(save_dir, package_name, GZB_Version)

        # Generate STl files
        utils.copy_occs(root)
        utils.export_stl(design, save_dir, components)

        ui.messageBox(msg, title)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
