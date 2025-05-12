
from xml.etree.ElementTree import Element, SubElement
from ..utils import utils 

class Sensors_GZS:
    @staticmethod    
    def make_L2D(reference_link_name):
        # Initialize the sensor element for L2D LiDAR
        gazebo = Element('gazebo', reference=reference_link_name)
        reference_link_name=utils.extract_prefix(reference_link_name)
        sensor = SubElement(gazebo, 'sensor', name=f"{reference_link_name}_L2D", type="gpu_lidar")

        lidar = SubElement(sensor, 'lidar')
        
        SubElement(sensor, 'always_on').text = f"${{L2D_params['{reference_link_name}_L2D']['always_on']}}"
        SubElement(sensor, 'visualize').text = f"${{L2D_params['{reference_link_name}_L2D']['visualize']}}"
        SubElement(sensor, 'update_rate').text = f"${{L2D_params['{reference_link_name}_L2D']['update_rate']}}"
        SubElement(sensor, 'topic').text = f"${{L2D_params['{reference_link_name}_L2D']['topic']}}"
        SubElement(sensor, 'gz_frame_id').text = f"${{L2D_params['{reference_link_name}_L2D']['frame_name']}}"
        
        scan = SubElement(lidar, 'scan')  
        horizontal = SubElement(scan, 'horizontal')
        SubElement(horizontal, 'samples').text = f"${{L2D_params['{reference_link_name}_L2D']['horizontal']['samples']}}"
        SubElement(horizontal, 'resolution').text = f"${{L2D_params['{reference_link_name}_L2D']['horizontal']['resolution']}}"
        SubElement(horizontal, 'min_angle').text = f"${{L2D_params['{reference_link_name}_L2D']['horizontal']['min_angle']}}"
        SubElement(horizontal, 'max_angle').text = f"${{L2D_params['{reference_link_name}_L2D']['horizontal']['max_angle']}}"

        vertical = SubElement(scan, 'vertical')
        SubElement(vertical, 'samples').text = '1'
        SubElement(vertical, 'resolution').text = '1'
        SubElement(vertical, 'min_angle').text = '0'
        SubElement(vertical, 'max_angle').text = '0'

        range_elem = SubElement(lidar, 'range')
        SubElement(range_elem, 'min').text = f"${{L2D_params['{reference_link_name}_L2D']['range']['min']}}"
        SubElement(range_elem, 'max').text = f"${{L2D_params['{reference_link_name}_L2D']['range']['max']}}"
        SubElement(range_elem, 'resolution').text = f"${{L2D_params['{reference_link_name}_L2D']['range']['resolution']}}"

        noise = SubElement(lidar, 'noise')
        SubElement(noise, 'type').text = f"${{L2D_params['{reference_link_name}_L2D']['noise']['type']}}"
        SubElement(noise, 'mean').text = f"${{L2D_params['{reference_link_name}_L2D']['noise']['mean']}}"
        SubElement(noise, 'stddev').text = f"${{L2D_params['{reference_link_name}_L2D']['noise']['stddev']}}"

        return_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        return return_xml
    
    @staticmethod    
    def make_L3D(reference_link_name):
        # Initialize the sensor element for L3D LiDAR
        gazebo = Element('gazebo', reference=reference_link_name)
        
        reference_link_name=utils.extract_prefix(reference_link_name)
        sensor = SubElement(gazebo, 'sensor', name=f"{reference_link_name}_L3D", type="gpu_lidar")
        lidar = SubElement(sensor, 'ray')
        
        SubElement(sensor, 'always_on').text = f"${{L3D_params['{reference_link_name}_L3D']['always_on']}}"
        SubElement(sensor, 'visualize').text = f"${{L3D_params['{reference_link_name}_L3D']['visualize']}}"
        SubElement(sensor, 'update_rate').text = f"${{L3D_params['{reference_link_name}_L3D']['update_rate']}}"
        SubElement(sensor, 'topic').text = f"${{L3D_params['{reference_link_name}_L3D']['topic']}}"
        SubElement(sensor, 'gz_frame_id').text = f"${{L3D_params['{reference_link_name}_L3D']['frame_name']}}"
        
        scan = SubElement(lidar, 'scan')  
        horizontal = SubElement(scan, 'horizontal')
        SubElement(horizontal, 'samples').text = f"${{L3D_params['{reference_link_name}_L3D']['horizontal']['samples']}}"
        SubElement(horizontal, 'resolution').text = f"${{L3D_params['{reference_link_name}_L3D']['horizontal']['resolution']}}"
        SubElement(horizontal, 'min_angle').text = f"${{L3D_params['{reference_link_name}_L3D']['horizontal']['min_angle']}}"
        SubElement(horizontal, 'max_angle').text = f"${{L3D_params['{reference_link_name}_L3D']['horizontal']['max_angle']}}"

        vertical = SubElement(scan, 'vertical')
        SubElement(vertical, 'samples').text = f"${{L3D_params['{reference_link_name}_L3D']['horizontal']['samples']}}"
        SubElement(vertical, 'resolution').text = f"${{L3D_params['{reference_link_name}_L3D']['vertical']['resolution']}}"
        SubElement(vertical, 'min_angle').text = f"${{L3D_params['{reference_link_name}_L3D']['vertical']['min_angle']}}"
        SubElement(vertical, 'max_angle').text = f"${{L3D_params['{reference_link_name}_L3D']['vertical']['max_angle']}}"

        range_elem = SubElement(lidar, 'range')
        SubElement(range_elem, 'min').text = f"${{L3D_params['{reference_link_name}_L3D']['range']['min']}}"
        SubElement(range_elem, 'max').text = f"${{L3D_params['{reference_link_name}_L3D']['range']['max']}}"
        SubElement(range_elem, 'resolution').text = f"${{L3D_params['{reference_link_name}_L3D']['range']['resolution']}}"

        noise = SubElement(lidar, 'noise')
        SubElement(noise, 'type').text = f"${{L3D_params['{reference_link_name}_L3D']['noise']['type']}}"
        SubElement(noise, 'mean').text = f"${{L3D_params['{reference_link_name}_L3D']['noise']['mean']}}"
        SubElement(noise, 'stddev').text = f"${{L3D_params['{reference_link_name}_L3D']['noise']['stddev']}}"

        return_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        return return_xml
    
    @staticmethod    
    def make_DCAM(reference_link_name):
        # Initialize the sensor element for depth camera
        gazebo = Element('gazebo', reference=reference_link_name)
        
        reference_link_name=utils.extract_prefix(reference_link_name)
        sensor = SubElement(gazebo, 'sensor', name=f"{reference_link_name}_DCAM", type="rgbd_camera")

        SubElement(sensor, 'always_on').text = f"${{DCAM_params['{reference_link_name}_DCAM']['always_on']}}"
        SubElement(sensor, 'update_rate').text = f"${{DCAM_params['{reference_link_name}_DCAM']['update_rate']}}"
        SubElement(sensor, 'topic').text = f"${{DCAM_params['{reference_link_name}_DCAM']['image_topic']}}"
        SubElement(sensor, 'gz_frame_id').text = f"${{DCAM_params['{reference_link_name}_DCAM']['frame_name']}}"

        camera = SubElement(sensor, 'camera', name=f"{reference_link_name}_DCAM")
        SubElement(camera, 'horizontal_fov').text = f"${{DCAM_params['{reference_link_name}_DCAM']['horizontal_fov']}}"
        SubElement(camera, 'camera_info_topic').text = f"${{DCAM_params['{reference_link_name}_DCAM']['info_topic']}}"

        lens = SubElement(camera, 'lens')
        intrinsics = SubElement(lens, 'intrinsics')
        SubElement(intrinsics, 'fx').text = f"${{DCAM_params['{reference_link_name}_DCAM']['intrinsics']['fx']}}"
        SubElement(intrinsics, 'fy').text = f"${{DCAM_params['{reference_link_name}_DCAM']['intrinsics']['fy']}}"
        SubElement(intrinsics, 'cx').text = f"${{DCAM_params['{reference_link_name}_DCAM']['intrinsics']['cx']}}"
        SubElement(intrinsics, 'cy').text = f"${{DCAM_params['{reference_link_name}_DCAM']['intrinsics']['cy']}}"
        SubElement(intrinsics, 's').text = f"${{DCAM_params['{reference_link_name}_DCAM']['intrinsics']['s']}}"

        distortion = SubElement(camera, 'distortion')
        SubElement(distortion, 'k1').text = f"${{DCAM_params['{reference_link_name}_DCAM']['distortion']['k1']}}"
        SubElement(distortion, 'k2').text = f"${{DCAM_params['{reference_link_name}_DCAM']['distortion']['k2']}}"
        SubElement(distortion, 'k3').text = f"${{DCAM_params['{reference_link_name}_DCAM']['distortion']['k3']}}"
        SubElement(distortion, 'p1').text = f"${{DCAM_params['{reference_link_name}_DCAM']['distortion']['p1']}}"
        SubElement(distortion, 'p2').text = f"${{DCAM_params['{reference_link_name}_DCAM']['distortion']['p2']}}"

        image = SubElement(camera, 'image')
        SubElement(image, 'width').text = f"${{DCAM_params['{reference_link_name}_DCAM']['image']['width']}}"
        SubElement(image, 'height').text = f"${{DCAM_params['{reference_link_name}_DCAM']['image']['height']}}"
        SubElement(image, 'format').text = f"${{DCAM_params['{reference_link_name}_DCAM']['image']['format']}}"

        clip = SubElement(camera, 'clip')
        SubElement(clip, 'near').text = f"${{DCAM_params['{reference_link_name}_DCAM']['clip']['near']}}"
        SubElement(clip, 'far').text = f"${{DCAM_params['{reference_link_name}_DCAM']['clip']['far']}}"

        depth_camera = SubElement(camera, 'depth_camera')
        depth_clip = SubElement(depth_camera, 'clip')
        SubElement(depth_clip, 'near').text = f"${{DCAM_params['{reference_link_name}_DCAM']['depth_camera']['clip']['near']}}"
        SubElement(depth_clip, 'far').text = f"${{DCAM_params['{reference_link_name}_DCAM']['depth_camera']['clip']['far']}}"

        noise = SubElement(camera, 'noise')
        SubElement(noise, 'type').text = f"${{DCAM_params['{reference_link_name}_DCAM']['noise']['type']}}"
        SubElement(noise, 'mean').text = f"${{DCAM_params['{reference_link_name}_DCAM']['noise']['mean']}}"
        SubElement(noise, 'stddev').text = f"${{DCAM_params['{reference_link_name}_DCAM']['noise']['stddev']}}"

        return_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        return return_xml
    
    @staticmethod    
    def make_CAM(reference_link_name):
        # Initialize the sensor element for monocular camera
        gazebo = Element('gazebo', reference=reference_link_name)
        
        reference_link_name=utils.extract_prefix(reference_link_name)
        sensor = SubElement(gazebo, 'sensor', name=f"{reference_link_name}_CAM", type="camera")

        SubElement(sensor, 'always_on').text = f"${{CAM_params['{reference_link_name}_CAM']['always_on']}}"
        SubElement(sensor, 'update_rate').text = f"${{CAM_params['{reference_link_name}_CAM']['update_rate']}}"
        SubElement(sensor, 'pose').text = f"${{CAM_params['{reference_link_name}_CAM']['pose']}}"
        SubElement(sensor, 'topic').text = f"${{CAM_params['{reference_link_name}_CAM']['image_topic']}}"
        SubElement(sensor, 'gz_frame_id').text = f"${{CAM_params['{reference_link_name}_CAM']['frame_name']}}"

        camera = SubElement(sensor, 'camera', name=f"{reference_link_name}_CAM")
        SubElement(camera, 'horizontal_fov').text = f"${{CAM_params['{reference_link_name}_CAM']['horizontal_fov']}}"
        SubElement(camera, 'camera_info_topic').text = f"${{CAM_params['{reference_link_name}_CAM']['info_topic']}}"

        lens = SubElement(camera, 'lens')
        intrinsics = SubElement(lens, 'intrinsics')
        SubElement(intrinsics, 'fx').text = f"${{CAM_params['{reference_link_name}_CAM']['intrinsics']['fx']}}"
        SubElement(intrinsics, 'fy').text = f"${{CAM_params['{reference_link_name}_CAM']['intrinsics']['fy']}}"
        SubElement(intrinsics, 'cx').text = f"${{CAM_params['{reference_link_name}_CAM']['intrinsics']['cx']}}"
        SubElement(intrinsics, 'cy').text = f"${{CAM_params['{reference_link_name}_CAM']['intrinsics']['cy']}}"
        SubElement(intrinsics, 's').text = f"${{CAM_params['{reference_link_name}_CAM']['intrinsics']['s']}}"

        distortion = SubElement(camera, 'distortion')
        SubElement(distortion, 'k1').text = f"${{CAM_params['{reference_link_name}_CAM']['distortion']['k1']}}"
        SubElement(distortion, 'k2').text = f"${{CAM_params['{reference_link_name}_CAM']['distortion']['k2']}}"
        SubElement(distortion, 'k3').text = f"${{CAM_params['{reference_link_name}_CAM']['distortion']['k3']}}"
        SubElement(distortion, 'p1').text = f"${{CAM_params['{reference_link_name}_CAM']['distortion']['p1']}}"
        SubElement(distortion, 'p2').text = f"${{CAM_params['{reference_link_name}_CAM']['distortion']['p2']}}"

        image = SubElement(camera, 'image')
        SubElement(image, 'width').text = f"${{CAM_params['{reference_link_name}_CAM']['image']['width']}}"
        SubElement(image, 'height').text = f"${{CAM_params['{reference_link_name}_CAM']['image']['height']}}"
        SubElement(image, 'format').text = f"${{CAM_params['{reference_link_name}_CAM']['image']['format']}}"

        clip = SubElement(camera, 'clip')
        SubElement(clip, 'near').text = f"${{CAM_params['{reference_link_name}_CAM']['clip']['near']}}"
        SubElement(clip, 'far').text = f"${{CAM_params['{reference_link_name}_CAM']['clip']['far']}}"

        noise = SubElement(camera, 'noise')
        SubElement(noise, 'type').text = f"${{CAM_params['{reference_link_name}_CAM']['noise']['type']}}"
        SubElement(noise, 'mean').text = f"${{CAM_params['{reference_link_name}_CAM']['noise']['mean']}}"
        SubElement(noise, 'stddev').text = f"${{CAM_params['{reference_link_name}_CAM']['noise']['stddev']}}"

        return_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        return return_xml
    
    @staticmethod    
    def make_IMU(reference_link_name):
        # Initialize the sensor element for IMU
        gazebo = Element('gazebo', reference=reference_link_name)
        plugin =SubElement(gazebo,'plugin',filename="gz-sim-imu-system",name="gz::sim::systems::Imu")
        reference_link_name=utils.extract_prefix(reference_link_name)
        sensor = SubElement(gazebo, 'sensor', name=f"{reference_link_name}_IMU", type="imu")

        # Add always_on and update_rate elements, fetched from IMU_params
        SubElement(sensor, 'always_on').text = f"${{IMU_params['{reference_link_name}_IMU']['always_on']}}"
        SubElement(sensor, 'update_rate').text = f"${{IMU_params['{reference_link_name}_IMU']['update_rate']}}"
        SubElement(sensor, 'topic').text = f"${{IMU_params['{reference_link_name}_IMU']['topic']}}"

        # Create imu element
        imu = SubElement(sensor, 'imu')

        # Angular velocity setup, fetched dynamically from IMU_params
        angular_velocity = SubElement(imu, 'angular_velocity')
        for axis in ['x', 'y', 'z']:
            axis_elem = SubElement(angular_velocity, axis)
            noise = SubElement(axis_elem, 'noise', type="gaussian")
            SubElement(noise, 'mean').text = f"${{IMU_params['{reference_link_name}_IMU']['angular_velocity']['{axis}']['noise']['mean']}}"
            SubElement(noise, 'stddev').text = f"${{IMU_params['{reference_link_name}_IMU']['angular_velocity']['{axis}']['noise']['stddev']}}"
            # SubElement(noise, 'bias_mean').text = f"${{IMU_params['{reference_link_name}_IMU']['angular_velocity']['{axis}']['bias_mean']}}"
            # SubElement(noise, 'bias_stddev').text = f"${{IMU_params['{reference_link_name}_IMU']['angular_velocity']['{axis}']['bias_stddev']}}"
            # SubElement(noise, 'dynamic_bias_stddev').text = f"${{IMU_params['{reference_link_name}_IMU']['angular_velocity']['{axis}']['dynamic_bias_stddev']}}"
            # SubElement(noise, 'dynamic_bias_correlation_time').text = f"${{IMU_params['{reference_link_name}_IMU']['angular_velocity']['{axis}']['dynamic_bias_correlation_time']}}"
            # SubElement(noise, 'precision').text = f"${{IMU_params['{reference_link_name}_IMU']['angular_velocity']['{axis}']['precision']}}"

        # Linear acceleration setup, fetched dynamically from IMU_params
        linear_acceleration = SubElement(imu, 'linear_acceleration')
        for axis in ['x', 'y', 'z']:
            axis_elem = SubElement(linear_acceleration, axis)
            noise = SubElement(axis_elem, 'noise', type="gaussian")
            SubElement(noise, 'mean').text = f"${{IMU_params['{reference_link_name}_IMU']['linear_acceleration']['{axis}']['noise']['mean']}}"
            SubElement(noise, 'stddev').text = f"${{IMU_params['{reference_link_name}_IMU']['linear_acceleration']['{axis}']['noise']['stddev']}}"
            # SubElement(noise, 'bias_mean').text = f"${{IMU_params['{reference_link_name}_IMU']['linear_acceleration']['{axis}']['bias_mean']}}"
            # SubElement(noise, 'bias_stddev').text = f"${{IMU_params['{reference_link_name}_IMU']['linear_acceleration']['{axis}']['bias_stddev']}}"
            # SubElement(noise, 'dynamic_bias_stddev').text = f"${{IMU_params['{reference_link_name}_IMU']['linear_acceleration']['{axis}']['dynamic_bias_stddev']}}"
            # SubElement(noise, 'dynamic_bias_correlation_time').text = f"${{IMU_params['{reference_link_name}_IMU']['linear_acceleration']['{axis}']['dynamic_bias_correlation_time']}}"
            # SubElement(noise, 'precision').text = f"${{IMU_params['{reference_link_name}_IMU']['linear_acceleration']['{axis}']['precision']}}"

        return_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        return return_xml
    
    @staticmethod
    def make_GPS(reference_link_name):
        # Create the root <gazebo> element with a reference link
        gazebo = Element('gazebo', reference=reference_link_name)
        
        reference_link_name=utils.extract_prefix(reference_link_name)
        sensor = SubElement(gazebo, 'sensor', name=f"{reference_link_name}_GPS", type="navsat")

        # Add <always_on> and <update_rate> using GPS_params
        always_on = SubElement(sensor, 'always_on')
        always_on.text = f"${{GPS_params['{reference_link_name}_GPS']['always_on']}}"

        update_rate = SubElement(sensor, 'update_rate')
        update_rate.text = f"${{GPS_params['{reference_link_name}_GPS']['update_rate']}}"

        # Position sensing
        position_sensing = SubElement(sensor, 'position_sensing')
        horizontal = SubElement(position_sensing, 'horizontal')
        h_noise = SubElement(horizontal, 'noise', type="gaussian")
        SubElement(h_noise, 'mean').text = f"${{GPS_params['{reference_link_name}_GPS']['pos']['noise']['horizontal']['mean']}}"
        SubElement(h_noise, 'stddev').text = f"${{GPS_params['{reference_link_name}_GPS']['pos']['noise']['horizontal']['stddev']}}"

        vertical = SubElement(position_sensing, 'vertical')
        v_noise = SubElement(vertical, 'noise', type="gaussian")
        SubElement(v_noise, 'mean').text = f"${{GPS_params['{reference_link_name}_GPS']['pos']['noise']['vertical']['mean']}}"
        SubElement(v_noise, 'stddev').text = f"${{GPS_params['{reference_link_name}_GPS']['pos']['noise']['vertical']['stddev']}}"

        # Velocity sensing
        velocity_sensing = SubElement(sensor, 'velocity_sensing')
        horizontal_vel = SubElement(velocity_sensing, 'horizontal')
        h_vel_noise = SubElement(horizontal_vel, 'noise', type="gaussian")
        SubElement(h_vel_noise, 'mean').text = f"${{GPS_params['{reference_link_name}_GPS']['vel']['noise']['horizontal']['mean']}}"
        SubElement(h_vel_noise, 'stddev').text = f"${{GPS_params['{reference_link_name}_GPS']['vel']['noise']['horizontal']['stddev']}}"

        vertical_vel = SubElement(velocity_sensing, 'vertical')
        v_vel_noise = SubElement(vertical_vel, 'noise', type="gaussian")
        SubElement(v_vel_noise, 'mean').text = f"${{GPS_params['{reference_link_name}_GPS']['vel']['noise']['vertical']['mean']}}"
        SubElement(v_vel_noise, 'stddev').text = f"${{GPS_params['{reference_link_name}_GPS']['vel']['noise']['vertical']['stddev']}}"

        # Return pretty-printed XML without header
        return_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        return return_xml

    @staticmethod
    def make_ODOM(reference_link_name):
        # Creating the root gazebo element
        gazebo = Element('gazebo')
        plugin = SubElement(gazebo, 'plugin', filename="libgz-sim-odometry-publisher-system", 
                            name="gz::sim::systems::OdometryPublisher")
        odom_frame = SubElement(plugin, 'odom_frame')
        odom_frame.text = f"${{ODOM_params['{reference_link_name}_ODOM']['odom_frame']}}"

        robot_base_frame = SubElement(plugin, 'robot_base_frame')
        robot_base_frame.text = f"${{ODOM_params['{reference_link_name}_ODOM']['robot_base_frame']}}"

        odom_topic = SubElement(plugin, 'odom_topic')
        odom_topic.text = f"${{ODOM_params['{reference_link_name}_ODOM']['odom_topic']}}"

        tf_topic = SubElement(plugin, 'tf_topic')
        tf_topic.text = f"${{ODOM_params['{reference_link_name}_ODOM']['tf_topic']}}"

        dimensions = SubElement(plugin, 'dimensions')
        dimensions.text = f"${{ODOM_params['{reference_link_name}_ODOM']['dimensions']}}"

        odom_publish_frequency = SubElement(plugin, 'odom_publish_frequency')
        odom_publish_frequency.text = f"${{ODOM_params['{reference_link_name}_ODOM']['odom_publish_frequency']}}"

        return_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        return return_xml





