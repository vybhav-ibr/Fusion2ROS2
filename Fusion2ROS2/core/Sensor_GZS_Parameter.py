from ..utils import utils

class Sensor_GZS_Parameter:

    @staticmethod
    def make_L2D(reference_link_name,sensor_type):
        return {
                'always_on':'true',
                'visualize':'false',
                'pose': '0.0 0 0.0 0 0 0',
                'update_rate': 15,
                'topic':f"{utils.extract_prefix(reference_link_name)}/scan",
                'frame_name':reference_link_name,
                'horizontal': {
                    'samples': 900,
                    'resolution': 1,
                    'min_angle': -3.1415,
                    'max_angle': 3.1415
                },
                'range': {
                    'min': 0.10,
                    'max': 60,
                    'resolution': 1
                },
                'noise': {
                    'type': 'gaussian',
                    'mean': 0.0,
                    'stddev': 0.01
                },

            }

    @staticmethod
    def make_L3D(reference_link_name,sensor_type):
        return{
                'always_on':'true',
                'visualize':'false',
                'pose': '0.0 0 0.0 0 0 0',
                'update_rate': 15,
                'topic':f"{utils.extract_prefix(reference_link_name)}",
                'frame_name': reference_link_name,
                'horizontal': {
                    'samples': 900,
                    'resolution': 1,
                    'min_angle': -3.1415,
                    'max_angle': 3.1415,
                },
                'vertical': {
                    'samples': 16,
                    'resolution': 1,
                    'min_angle': -0.2617,
                    'max_angle': 0.2617
                },
                'range': {
                    'min': 0.10,
                    'max': 60,
                    'resolution': 1
                },
                'noise': {
                    'type': 'gaussian',
                    'mean': 0.0,
                    'stddev': 0.01
                }
            }
        
    @staticmethod
    def make_DCAM(reference_link_name,sensor_type):
        return {
            "always_on": True,
            "update_rate": 15,
            "pose": "0 0 0 0 0 0",
            "horizontal_fov": 1.46608,
            'frame_name': reference_link_name,
            "image_topic":f"{utils.extract_prefix(reference_link_name)}",
            "info_topic":f"{utils.extract_prefix(reference_link_name)}/camera_info",
            'intrinsics':
            {
                'fx':277,
                'fy':277,
                'cx':360,
                'cy':320,
                's':0.0,
            },
            "distortion":
            {
                'k1':0.0,
                'k2':0.0,
                'k3':0.0,
                'p1':0.0,
                'p2':0.0,                                                                            
            },
            "image": {
                "width": 720,
                "height": 640,
                "format": "B8G8R8"
            },
            "clip": {
                "near": 0.1,
                "far": 20
            },
            'depth_camera':{
                "clip": {
                    "near": 0.1,
                    "far": 20
                },
            },
            "noise":
            {
                "type":'gaussian',
                "mean":0.01,
                "stddev":0.01,
            }
        }

    @staticmethod
    def make_CAM(reference_link_name, sensor_type):
        return {
                "always_on": True,
                "update_rate": 15,
                "pose": "0 0 0 0 0 0",
                "horizontal_fov": 1.46608,
                'frame_name': reference_link_name,
                "image_topic":f"{utils.extract_prefix(reference_link_name)}/image_raw",
                "info_topic":f"{utils.extract_prefix(reference_link_name)}/camera_info",
                'intrinsics':
                {
                    'fx':277,
                    'fy':277,
                    'cx':360,
                    'cy':320,
                    's':0.0,
                },
                "distortion":
                {
                    'k1':0.0,
                    'k2':0.0,
                    'k3':0.0,
                    'p1':0.0,
                    'p2':0.0,                                                                            
                },
                "image": {
                    "width": 720,
                    "height": 640,
                    "format": "B8G8R8"
                },
                "clip": {
                    "near": 0.05,
                    "far": 8
                },
                "noise":
                {
                    "type":'gaussian',
                    "mean":0.01,
                    "stddev":0.01,
                }
            }
    @staticmethod
    def make_IMU(reference_link_name,sensor_type):
        return {
                    "always_on": True,
                    "update_rate": 100,
                    'topic':f"{utils.extract_prefix(reference_link_name)}/imu",
                    'frame_name':reference_link_name,
                    "angular_velocity": {
                        "x": {
                            "noise": {
                                "mean": 0.0,
                                "stddev": 2e-4
                            }
                        },
                        "y": {
                            "noise": {
                                "mean": 0.0,
                                "stddev": 2e-4
                            }
                        },
                        "z": {
                            "noise": {
                                "mean": 0.0,
                                "stddev": 2e-4
                            }
                        }
                    },
                    "linear_acceleration": {
                        "x": {
                            "noise": {
                                "mean": 0.0,
                                "stddev": 1.7e-2
                            }
                        },
                        "y": {
                            "noise": {
                                "mean": 0.0,
                                "stddev": 1.7e-2
                            }
                        },
                        "z": {
                            "noise": {
                                "mean": 0.0,
                                "stddev": 1.7e-2
                            }
                        }
                    }
                }

    @staticmethod
    def make_GPS(reference_link_name,sensor_type):
        return {
                'always_on': True,
                'update_rate': 1.0,
                'topic':"/gps",
                'pos':{
                    'noise':{
                        'horizontal':{
                            'mean':0.1,
                            'stddev':0.0
                        },
                        'vertical':{
                            'mean':0.1,
                            'stddev':0.0
                        }
                    }
                },
                'vel':{
                    'noise':{
                        'horizontal':{
                            'mean':0.1,
                            'stddev':0.0
                        },
                        'vertical':{
                            'mean':0.1,
                            'stddev':0.0
                        }
                    }
                }
            }

    @staticmethod
    def make_ODOM(reference_link_name,sensor_type):
        return {
                    "odom_frame": "world",
                    "robot_base_frame":"base_link",
                    "odom_topic":'odom',
                    "tf_topic":'tf',
                    "odom_publish_frequency": 100.0,
            }
    
    @staticmethod
    def write_yaml(data, filename, indent=0):
        def write_line(file, line, indent):
            file.write('  ' * indent + line + '\n')

        def recurse(data, file, indent):
            if isinstance(data, dict):
                for key, value in data.items():
                    if isinstance(value, (dict, list)):
                        write_line(file, f"{key}:", indent)
                        recurse(value, file, indent + 1)
                    else:
                        write_line(file, f"{key}: {value}", indent)
            elif isinstance(data, list):
                for item in data:
                    if isinstance(item, (dict, list)):
                        write_line(file, f"-", indent)
                        recurse(item, file, indent + 1)
                    else:
                        write_line(file, f"- {item}", indent)

        with open(filename, 'w') as file:
            recurse(data, file, indent)
