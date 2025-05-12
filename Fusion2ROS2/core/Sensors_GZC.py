
from xml.etree.ElementTree import Element, SubElement
from ..utils import utils  

class Sensors_GZC:
    def __init__(self, name):
        """
        Attributes
        ----------
        name: str
            name of the joint
        type: str
            type of the joint(ex: rev)
        xyz: [x, y, z]
            coordinate of the joint
        axis: [x, y, z]
            coordinate of axis of the joint
        parent: str
            parent link
        child: str
            child link
        joint_xml: str
            generated xml describing about the joint
        tran_xml: str
            generated xml describing about the transmission
        """ 
        reference_the_link=True
    
    @staticmethod
    def make_L2D(reference_link_name):
                # Create the root gazebo element
        gazebo = Element('gazebo', reference=reference_link_name)
                
        # Create the sensor element
        reference_link_name=utils.extract_prefix(reference_link_name)
        sensor = SubElement(gazebo, 'sensor', name=f"{reference_link_name}_L2D", type="ray")
                
        # Add sensor children
        SubElement(sensor, 'always_on').text = f"${{L2D_params['{reference_link_name}_L2D']['always_on']}}"
        SubElement(sensor, 'visualize').text = f"${{L2D_params['{reference_link_name}_L2D']['visualize']}}"
        SubElement(sensor, 'pose').text = f"${{L2D_params['{reference_link_name}_L2D']['pose']}}"
        SubElement(sensor, 'update_rate').text = f"${{L2D_params['{reference_link_name}_L2D']['update_rate']}}"

        # Add ray element and scan properties
        ray = SubElement(sensor, 'ray')
        scan = SubElement(ray, 'scan')
        horizontal = SubElement(scan, 'horizontal')
        SubElement(horizontal, 'samples').text = f"${{L2D_params['{reference_link_name}_L2D']['horizontal']['samples']}}"
        SubElement(horizontal, 'resolution').text = f"${{L2D_params['{reference_link_name}_L2D']['horizontal']['resolution']}}"
        SubElement(horizontal, 'min_angle').text = f"${{L2D_params['{reference_link_name}_L2D']['horizontal']['min_angle']}}"
        SubElement(horizontal, 'max_angle').text = f"${{L2D_params['{reference_link_name}_L2D']['horizontal']['max_angle']}}"

        # Add range element
        range_elem = SubElement(ray, 'range')
        SubElement(range_elem, 'min').text = f"${{L2D_params['{reference_link_name}_L2D']['range']['min']}}"
        SubElement(range_elem, 'max').text = f"${{L2D_params['{reference_link_name}_L2D']['range']['max']}}"
        SubElement(range_elem, 'resolution').text = f"${{L2D_params['{reference_link_name}_L2D']['range']['resolution']}}"

        # Add noise element
        noise = SubElement(ray, 'noise')
        SubElement(noise, 'type').text = f"${{L2D_params['{reference_link_name}_L2D']['noise']['type']}}"
        SubElement(noise, 'mean').text = f"${{L2D_params['{reference_link_name}_L2D']['noise']['mean']}}"
        SubElement(noise, 'stddev').text = f"${{L2D_params['{reference_link_name}_L2D']['noise']['stddev']}}"

        # Add plugin element
        plugin = SubElement(sensor, 'plugin', name=reference_link_name, filename="libgazebo_ros_ray_sensor.so")

        # Add the ros element and remappings
        ros = SubElement(plugin, 'ros')
        SubElement(ros, 'namespace').text = f"${{L2D_params['{reference_link_name}_L2D']['ros']['namespace']}}"
        SubElement(ros, 'remapping').text = f"${{L2D_params['{reference_link_name}_L2D']['ros']['remapping']}}"

        # Add other plugin parameters
        SubElement(plugin, 'output_type').text = f"${{L2D_params['{reference_link_name}_L2D']['output_type']}}"
        SubElement(plugin, 'frame_name').text = f"${{L2D_params['{reference_link_name}_L2D']['frame_name']}}"
       
        # Return the formatted XML string (prettified)
        return_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        return return_xml

    
    @staticmethod    
    def make_L3D(reference_link_name):
        gazebo = Element('gazebo', reference=reference_link_name)
        reference_link_name=utils.extract_prefix(reference_link_name)
        # Create the sensor element
        sensor = SubElement(gazebo, 'sensor', name=f"{reference_link_name}_L3D", type="ray")
        
        # Add sensor children using the lidar_params dictionary
        SubElement(sensor, 'pose').text = f"${{L3D_params['{reference_link_name}_L3D']['pose']}}"
        SubElement(sensor, 'visualize').text = f"${{L3D_params['{reference_link_name}_L3D']['visualize']}}"
        SubElement(sensor, 'update_rate').text = f"${{L3D_params['{reference_link_name}_L3D']['update_rate']}}"

        # Add ray element and scan properties
        ray = SubElement(sensor, 'ray')
        scan = SubElement(ray, 'scan')

        # Add horizontal scan
        horizontal = SubElement(scan, 'horizontal')
        SubElement(horizontal, 'samples').text = f"${{L3D_params['{reference_link_name}_L3D']['horizontal']['samples']}}"
        SubElement(horizontal, 'resolution').text = f"${{L3D_params['{reference_link_name}_L3D']['horizontal']['resolution']}}"
        SubElement(horizontal, 'min_angle').text = f"${{L3D_params['{reference_link_name}_L3D']['horizontal']['min_angle']}}"
        SubElement(horizontal, 'max_angle').text = f"${{L3D_params['{reference_link_name}_L3D']['horizontal']['max_angle']}}"

        # Add vertical scan
        vertical = SubElement(scan, 'vertical')
        SubElement(vertical, 'samples').text = f"${{L3D_params['{reference_link_name}_L3D']['vertical']['samples']}}"
        SubElement(vertical, 'resolution').text = f"${{L3D_params['{reference_link_name}_L3D']['vertical']['resolution']}}"
        SubElement(vertical, 'min_angle').text = f"${{L3D_params['{reference_link_name}_L3D']['vertical']['min_angle']}}"
        SubElement(vertical, 'max_angle').text = f"${{L3D_params['{reference_link_name}_L3D']['vertical']['max_angle']}}"

        # Add range element
        range_elem = SubElement(ray, 'range')
        SubElement(range_elem, 'min').text = f"${{L3D_params['{reference_link_name}_L3D']['range']['min']}}"
        SubElement(range_elem, 'max').text = f"${{L3D_params['{reference_link_name}_L3D']['range']['max']}}"
        SubElement(range_elem, 'resolution').text = f"${{L3D_params['{reference_link_name}_L3D']['range']['resolution']}}"

        # Add noise element
        noise = SubElement(ray, 'noise')
        SubElement(noise, 'type').text = f"${{L3D_params['{reference_link_name}_L3D']['noise']['type']}}"
        SubElement(noise, 'mean').text = f"${{L3D_params['{reference_link_name}_L3D']['noise']['mean']}}"
        SubElement(noise, 'stddev').text = f"${{L3D_params['{reference_link_name}_L3D']['noise']['stddev']}}"

        # Add plugin element
        plugin = SubElement(sensor, 'plugin', name=reference_link_name, filename="libgazebo_ros_ray_sensor.so")

        # Add the ros element and remappings
        ros = SubElement(plugin, 'ros')
        SubElement(ros, 'namespace').text = f"${{L3D_params['{reference_link_name}_L3D']['ros']['namespace']}}"
        SubElement(ros, 'remapping').text = f"${{L3D_params['{reference_link_name}_L3D']['ros']['remapping']}}"

        # Add other plugin parameters
        SubElement(plugin, 'tf_prefix').text = f"${{L3D_params['{reference_link_name}_L3D']['tf_prefix']}}"
        SubElement(plugin, 'frame_name').text = f"${{L3D_params['{reference_link_name}_L3D']['frame_name']}}"
        SubElement(plugin, 'organize_cloud').text = f"${{L3D_params['{reference_link_name}_L3D']['organize_cloud']}}"
        SubElement(plugin, 'min_range').text = f"${{L3D_params['{reference_link_name}_L3D']['min_range']}}"
        SubElement(plugin, 'max_range').text = f"${{L3D_params['{reference_link_name}_L3D']['max_range']}}"
        SubElement(plugin, 'gaussian_noise').text = f"${{L3D_params['{reference_link_name}_L3D']['gaussian_noise']}}"
                
        # Return the formatted XML string (prettified)
        return_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        return return_xml
        
    @staticmethod    
    def make_DCAM(reference_link_name):
        gazebo = Element('gazebo', reference=reference_link_name)
        reference_link_name=utils.extract_prefix(reference_link_name)
        # Create the sensor element
        sensor = SubElement(gazebo, 'sensor', name=reference_link_name, type='depth')

        # Replace all parameters with DCAM_params
        SubElement(sensor, 'always_on').text = f"${{DCAM_params['{reference_link_name}_DCAM']['always_on']}}"
        SubElement(sensor, 'update_rate').text = f"${{DCAM_params['{reference_link_name}_DCAM']['update_rate']}}"
        SubElement(sensor, 'pose').text = f"${{DCAM_params['{reference_link_name}_DCAM']['pose']}}"

        # Add the camera element with parameters
        camera = SubElement(sensor, 'camera', name=f"{reference_link_name}_DCAM")
        SubElement(camera, 'horizontal_fov').text = f"${{DCAM_params['{reference_link_name}_DCAM']['horizontal_fov']}}"

        # Add the image element with parameters
        image = SubElement(camera, 'image')
        SubElement(image, 'width').text = f"${{DCAM_params['{reference_link_name}_DCAM']['image']['width']}}"
        SubElement(image, 'height').text = f"${{DCAM_params['{reference_link_name}_DCAM']['image']['height']}}"
        SubElement(image, 'format').text = f"${{DCAM_params['{reference_link_name}_DCAM']['image']['format']}}"

        # Clip parameters
        clip = SubElement(camera, 'clip')
        SubElement(clip, 'near').text = f"${{DCAM_params['{reference_link_name}_DCAM']['clip']['near']}}"
        SubElement(clip, 'far').text = f"${{DCAM_params['{reference_link_name}_DCAM']['clip']['far']}}"

        distortion = SubElement(camera, 'distortion')
        SubElement(distortion, 'k1').text = f"${{DCAM_params['{reference_link_name}_DCAM']['distortion']['k1']}}"
        SubElement(distortion, 'k2').text = f"${{DCAM_params['{reference_link_name}_DCAM']['distortion']['k2']}}"
        SubElement(distortion, 'k3').text = f"${{DCAM_params['{reference_link_name}_DCAM']['distortion']['k3']}}"
        SubElement(distortion, 'p1').text = f"${{DCAM_params['{reference_link_name}_DCAM']['distortion']['p1']}}"
        SubElement(distortion, 'p2').text = f"${{DCAM_params['{reference_link_name}_DCAM']['distortion']['p2']}}"

        # Add plugin element
        plugin = SubElement(sensor, 'plugin', name=reference_link_name, filename="libgazebo_ros_camera.so")

        # ROS remappings
        ros = SubElement(plugin, 'ros')
        SubElement(ros, 'namespace').text = f"${{DCAM_params['{reference_link_name}_DCAM']['ros']['namespace']}}"
        SubElement(ros, 'remapping').text = f"${{DCAM_params['{reference_link_name}_DCAM']['ros']['remappings'][0]}}"
        SubElement(ros, 'remapping').text = f"${{DCAM_params['{reference_link_name}_DCAM']['ros']['remappings'][1]}}"
        SubElement(ros, 'remapping').text = f"${{DCAM_params['{reference_link_name}_DCAM']['ros']['remappings'][2]}}"
        SubElement(ros, 'remapping').text = f"${{DCAM_params['{reference_link_name}_DCAM']['ros']['remappings'][3]}}"
        SubElement(ros, 'remapping').text = f"${{DCAM_params['{reference_link_name}_DCAM']['ros']['remappings'][4]}}"

        # Plugin parameters
        SubElement(plugin, 'camera_name').text = f"{reference_link_name}_DCAM"
        SubElement(plugin, 'frame_name').text = f"${{DCAM_params['{reference_link_name}_DCAM']['frame_name']}}"
        SubElement(plugin, 'hack_baseline').text = f"${{DCAM_params['{reference_link_name}_DCAM']['hack_baseline']}}"
        SubElement(plugin, 'min_depth').text = f"${{DCAM_params['{reference_link_name}_DCAM']['min_depth']}}"
        SubElement(plugin, 'max_depth').text = f"${{DCAM_params['{reference_link_name}_DCAM']['max_depth']}}"

        # Use the utils.prettify() function to format the XML, and store it as a string
        return_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        return return_xml
    
    @staticmethod
    def make_CAM(reference_link_name):
        gazebo = Element('gazebo', reference=reference_link_name)
        reference_link_name=utils.extract_prefix(reference_link_name)
        # Create the sensor element
        sensor = SubElement(gazebo, 'sensor', type='camera', name=reference_link_name)
        
        # Add sensor children using CAM parameters
        SubElement(sensor, 'always_on').text = f"${{CAM_params['{reference_link_name}_CAM']['always_on']}}"
        SubElement(sensor, 'update_rate').text = f"${{CAM_params['{reference_link_name}_CAM']['update_rate']}}"

        # Add camera element
        camera = SubElement(sensor, 'camera', name=reference_link_name)
        SubElement(camera, 'horizontal_fov').text = f"${{CAM_params['{reference_link_name}_CAM']['horizontal_fov']}}"

        # Add the image element
        image = SubElement(camera, 'image')
        SubElement(image, 'width').text = f"${{CAM_params['{reference_link_name}_CAM']['image']['width']}}"
        SubElement(image, 'height').text = f"${{CAM_params['{reference_link_name}_CAM']['image']['height']}}"
        SubElement(image, 'format').text = f"${{CAM_params['{reference_link_name}_CAM']['image']['format']}}"

        # Distortion parameters
        distortion = SubElement(camera, 'distortion')
        SubElement(distortion, 'k1').text = f"${{CAM_params['{reference_link_name}_CAM']['distortion']['k1']}}"
        SubElement(distortion, 'k2').text = f"${{CAM_params['{reference_link_name}_CAM']['distortion']['k2']}}"
        SubElement(distortion, 'k3').text = f"${{CAM_params['{reference_link_name}_CAM']['distortion']['k3']}}"
        SubElement(distortion, 'p1').text = f"${{CAM_params['{reference_link_name}_CAM']['distortion']['p1']}}"
        SubElement(distortion, 'p2').text = f"${{CAM_params['{reference_link_name}_CAM']['distortion']['p2']}}"

        # Plugin element
        plugin = SubElement(sensor, 'plugin', name=reference_link_name, filename="libgazebo_ros_camera.so")

        # ROS remappings
        ros = SubElement(plugin, 'ros')
        SubElement(ros,'namesoace').text= f"${{CAM_params['{reference_link_name}_CAM']['ros']['namespace']}}"
        SubElement(ros, 'remapping').text = f"${{CAM_params['{reference_link_name}_CAM']['ros']['remappings'][0]}}"
        SubElement(ros, 'remapping').text = f"${{CAM_params['{reference_link_name}_CAM']['ros']['remappings'][1]}}"

        # Other plugin parameters
        SubElement(plugin, 'camera_name').text = f"{reference_link_name}_CAM"
        SubElement(plugin, 'frame_name').text = f"${{CAM_params['{reference_link_name}_CAM']['frame_name']}}"
        SubElement(plugin, 'hack_baseline').text = f"${{CAM_params['{reference_link_name}_CAM']['hack_baseline']}}"
        
        # Return the formatted XML string (prettified)
        return_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        return return_xml
    
    @staticmethod    
    def make_IMU(reference_link_name):
        # Initialize the gazebo element
        gazebo = Element('gazebo', reference=reference_link_name)
        reference_link_name=utils.extract_prefix(reference_link_name)
        # Create the sensor element for IMU
        sensor = SubElement(gazebo, 'sensor', name=f"{reference_link_name}_IMU", type="imu")
        
        # Add sensor children dynamically using IMU_params
        SubElement(sensor, 'always_on').text = f"${{IMU_params['{reference_link_name}_IMU']['always_on']}}"
        SubElement(sensor, 'update_rate').text = f"${{IMU_params['{reference_link_name}_IMU']['update_rate']}}"

        # Add imu element with angular_velocity and linear_acceleration
        imu = SubElement(sensor, 'imu')

        # Angular velocity
        angular_velocity = SubElement(imu, 'angular_velocity')
        for axis in ['x', 'y', 'z']:
            axis_elem = SubElement(angular_velocity, axis)
            noise = SubElement(axis_elem, 'noise', type="gaussian")
            SubElement(noise, 'mean').text = f"${{IMU_params['{reference_link_name}_IMU']['angular_velocity']['{axis}']['noise']['mean']}}"
            SubElement(noise, 'stddev').text = f"${{IMU_params['{reference_link_name}_IMU']['angular_velocity']['{axis}']['noise']['stddev']}}"

        # Linear acceleration
        linear_acceleration = SubElement(imu, 'linear_acceleration')
        for axis in ['x', 'y', 'z']:
            axis_elem = SubElement(linear_acceleration, axis)
            noise = SubElement(axis_elem, 'noise', type="gaussian")
            SubElement(noise, 'mean').text = f"${{IMU_params['{reference_link_name}_IMU']['linear_acceleration']['{axis}']['noise']['mean']}}"
            SubElement(noise, 'stddev').text = f"${{IMU_params['{reference_link_name}_IMU']['linear_acceleration']['{axis}']['noise']['stddev']}}"

        # Add plugin element
        plugin = SubElement(sensor, 'plugin', name=reference_link_name,filename="libgazebo_ros_imu_sensor.so")

        # ROS namespace and remapping
        ros = SubElement(plugin, 'ros')
        SubElement(ros, 'namespace').text = f"${{IMU_params['{reference_link_name}_IMU']['ros']['namespace']}}"
        SubElement(ros, 'remapping').text = f"${{IMU_params['{reference_link_name}_IMU']['ros']['remapping']}}"

        # Plugin configuration
        SubElement(plugin, 'initial_orientation_as_reference').text = f"${{IMU_params['{reference_link_name}_IMU']['initial_orientation_as_reference']}}"
        
        # Return the formatted XML string (prettified)
        return_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        return return_xml

    
    @staticmethod
    def make_GPS(reference_link_name):
        gazebo = Element('gazebo', reference=reference_link_name)
        reference_link_name=utils.extract_prefix(reference_link_name)
        # Creating the <sensor> element for GPS
        sensor = SubElement(gazebo, 'sensor', name=f"{reference_link_name}_GPS", type="gps")

        # Add child elements dynamically using GPS_params
        SubElement(sensor, 'always_on').text = f"${{GPS_params['{reference_link_name}_GPS']['always_on']}}"

        SubElement(sensor, 'update_rate').text = f"${{GPS_params['{reference_link_name}_GPS']['update_rate']}}"

        # Create the plugin element for GPS
        plugin = SubElement(sensor, 'plugin', name=reference_link_name, filename="libgazebo_ros_gps_sensor.so")

        # Add the ros element and remapping for GPS
        ros = SubElement(plugin, 'ros')
        SubElement(ros, 'namespace').text = f"${{GPS_params['{reference_link_name}_GPS']['ros']['namespace']}}"
        SubElement(ros, 'remapping').text = f"${{GPS_params['{reference_link_name}_GPS']['ros']['remapping']}}"

        # Add frame_name for GPS plugin
        frame_name = SubElement(plugin, 'frame_name')
        frame_name.text = f"${{GPS_params['{reference_link_name}_GPS']['frame_name']}}"

        # Pretty printing the XML string
        return_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        return return_xml


    @staticmethod    
    def make_P3D(reference_link_name):
        gazebo = Element('gazebo')

        # Create the plugin element for P3D sensor
        plugin = SubElement(gazebo, 'plugin', name=reference_link_name, filename="libgazebo_ros_p3d.so")
        reference_link_name=utils.extract_prefix(reference_link_name)
        # Add the ros element for P3D sensor
        ros = SubElement(plugin, 'ros')

        # Add namespace and remapping dynamically using P3D_params
        namespace = SubElement(ros, 'namespace')
        namespace.text = f"${{P3D_params['{reference_link_name}_P3D']['ros']['namespace']}}"

        remapping = SubElement(ros, 'remapping')
        remapping.text = f"${{P3D_params['{reference_link_name}_P3D']['ros']['remapping']}}"

        # Add other parameters dynamically using P3D_params
        frame_name = SubElement(plugin, 'frame_name')
        frame_name.text = f"${{P3D_params['{reference_link_name}_P3D']['frame_name']}}"

        body_name = SubElement(plugin, 'body_name')
        body_name.text = f"${{P3D_params['{reference_link_name}_P3D']['body_name']}}"

        update_rate = SubElement(plugin, 'update_rate')
        update_rate.text = f"${{P3D_params['{reference_link_name}_P3D']['update_rate']}}"

        gaussian_noise = SubElement(plugin, 'gaussian_noise')
        gaussian_noise.text = f"${{P3D_params['{reference_link_name}_P3D']['gaussian_noise']}}"

        SubElement(plugin, 'xyzOffsets').text = f"${{P3D_params['{reference_link_name}_P3D']['xyzOffsets']}}"
        SubElement(plugin, 'rpyOffsets').text = f"${{P3D_params['{reference_link_name}_P3D']['rpyOffsets']}}"
 
        return_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        return return_xml
 
def make_sensors_dict(joints_dict, msg):
    sensors_dict={}
    for i in joints_dict:
        if 'L2D' in joints_dict[i]['child']:
            sensors_dict[joints_dict[i]['child']]='L2D'
        elif 'L3D' in joints_dict[i]['child']:
            sensors_dict[joints_dict[i]['child']]='L3D'
        elif 'DCAM' in joints_dict[i]['child']:
            sensors_dict[joints_dict[i]['child']]='DCAM'
        elif 'CAM' in joints_dict[i]['child']:
            sensors_dict[joints_dict[i]['child']]='CAM'
        elif 'IMU' in joints_dict[i]['child']:
            sensors_dict[joints_dict[i]['child']]='IMU'
        elif 'GPS' in joints_dict[i]['child']:
            sensors_dict[joints_dict[i]['child']]='GPS'
        elif 'P3D' in joints_dict[i]['child']:
            sensors_dict[joints_dict[i]['child']]='P3D'
    msg='Successfully create URDF file'
    return sensors_dict, msg
