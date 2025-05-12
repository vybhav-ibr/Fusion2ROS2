from ..utils import utils
class Sensor_GZC_Parameter:

    @staticmethod
    def make_L2D(reference_link_name,sensor_type):
        return {
                'always_on':'true',
                'visualize':'false',
                'pose': '0 0 0 0 0 0',
                'update_rate': 15,
                'horizontal': {
                    'samples': 900,
                    'resolution': 1.0,
                    'min_angle': -3.1415,
                    'max_angle': 3.1415
                },
                'range': {
                    'min': 0.12,
                    'max': 20,
                    'resolution': 1.0
                },
                'noise': {
                    'type': 'gaussian',
                    'mean': 0.01,
                    'stddev': 0.01
                },
                'ros':{
                    'namespace': "/",
                    'remapping': '/out:=scan'
                },
                'output_type':'sensor_msgs/LaserScan',
                'frame_name':reference_link_name
            }

    @staticmethod
    def make_L3D(reference_link_name,sensor_type):
        return {
                'type': 'ray',
                'pose': '0 0 0 0 0 0',
                'visualize':'false',
                'update_rate': 10,
                'horizontal': {
                    'samples': 900,
                    'resolution': 1,
                    'min_angle': -3.141592653589793,
                    'max_angle': 3.141592653589793
                },
                'vertical': {
                    'samples': 16,
                    'resolution': 1.0,
                    'min_angle': -0.2617,
                    'max_angle': 0.2617
                },
                'range': {
                    'min': 0.1,
                    'max': 131.0,
                    'resolution': 1.0
                },
                'noise': {
                    'type': 'gaussian',
                    'mean': 0.0,
                    'stddev': 0.0
                },
                'ros': {
                    'namespace': "/",
                    'remapping': '/out:=scan'
                },
                "tf_prefix": "",
                "frame_name": reference_link_name,
                "organize_cloud": False,
                "min_range": 0.1,
                "max_range": 130.0,
                'output_type':'sensor_msgs/PointCloud2',
                "gaussian_noise": 0.008
            }
        
    @staticmethod
    def make_DCAM(reference_link_name,sensor_type):
        return {
            "always_on": True,
            "update_rate": 15,
            "pose": "0 0 0 0 0 0",
            "horizontal_fov": 1.46608,
            "image": {
                "width": 720,
                "height": 640,
                "format": "B8G8R8"
            },
            "distortion": {
                "k1": 0.0,
                "k2": 0.0,
                "k3": 0.0,
                "p1": 0.0,
                "p2": 0.0,
            },
            "clip": {
                "near": 0.05,
                "far": 8
            },
            "ros": {
                "namespace": "/",
                "remappings": [
                    f"{reference_link_name}_DCAM/image_raw:=/image_raw",
                    f"{reference_link_name}_DCAM/image_depth:=/image_depth",
                    f"{reference_link_name}_DCAM/camera_info:=/camera_info",
                    f"{reference_link_name}_DCAM/camera_info_depth:=/depth_info",
                    f"{reference_link_name}_DCAM/points:=/points"
                ]
            },
            "frame_name": f"{utils.extract_prefix(reference_link_name)}_optical",
            "hack_baseline": 0.07,
            "min_depth": 0.05,
            "max_depth": 8.0
        }
    
    @staticmethod
    def make_CAM(reference_link_name, sensor_type):
        return {
                    "always_on": True,
                    "update_rate": 15.0,
                    "horizontal_fov": 1.46608,
                    "frame_name": reference_link_name,
                    "image": {
                        "width": 720,
                        "height": 640,
                        "format": "R8G8B8"
                    },
                    "distortion": {
                        "k1": 0.0,
                        "k2": 0.0,
                        "k3": 0.0,
                        "p1": 0.0,
                        "p2": 0.0,
                    },
                    "ros": {
                        "namespace": "/",
                        "remappings": [
                            "~/image_raw:=image_raw",
                            "~/camera_info:=camera_info"
                        ]
                    },
                    "hack_baseline": 0.2
                }
        


    @staticmethod
    def make_IMU(reference_link_name,sensor_type):
        return {
                    "always_on": True,
                    "update_rate": 100,
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
                    },
                    "ros": {
                        "namespace": "/",
                        "remapping": "~/out:=data"
                    },
                    "initial_orientation_as_reference": False
                }

    @staticmethod
    def make_GPS(reference_link_name,sensor_type):
        return {
                'always_on': True,
                'update_rate': 1.0,
                "ros": {
                    "namespace": "/",
                    "remapping": f"~/{reference_link_name}_GPS/out:=gps"
                },
                'frame_name':reference_link_name,
            }

    @staticmethod
    def make_P3D(reference_link_name,sensor_type):
        return {
                    "ros": {
                        "namespace": "/ground_truth",
                        "remapping": "/ground_truth/odom:=/ground_truth"
                    },
                    "frame_name": "world",
                    "body_name": "base_link",
                    "update_rate": 30.0,
                    "gaussian_noise": 0.0,
                    "xyzOffsets": [0, 0, 0],
                    "rpyOffsets": [0, 0, 0]
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
