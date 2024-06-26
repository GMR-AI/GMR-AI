import rclpy
from rclpy.node import Node
import rclpy.parameter

import subprocess
import os

class CoppeliaNode(Node):
    def __init__(self):
        super().__init__('coppelia_simulator')

        self.default_coppelia_root_dir = os.path.join(os.path.expanduser('~'), 'CoppeliaSim_Edu_V4_6_0_rev18_Ubuntu22_04')
        self.default_coppelia_scene_path = os.path.join(os.path.expanduser('~'), 'GMR-AI', 'sim', 'coppeliasim', 'GMR-Scene', 'GMR.ttt')
        self.default_coppelia_headless = False
        
        self.declare_parameter('coppelia_root_dir', self.default_coppelia_root_dir)
        self.coppelia_root_dir = self.get_parameter('coppelia_root_dir')
        
        self.declare_parameter('coppelia_scene_path', self.default_coppelia_scene_path)
        self.coppelia_scene_path = self.get_parameter('coppelia_scene_path')

        self.declare_parameter('coppelia_headless', self.default_coppelia_headless)
        self.coppelia_headless = self.get_parameter('coppelia_headless')

        self.get_logger().warn(f"Coppelia scene path: {self.coppelia_scene_path.get_parameter_value().string_value}")


    def run(self):
        if self.coppelia_headless.get_parameter_value().bool_value:
            subprocess.Popen([
                '/bin/bash', 
                os.path.join(self.coppelia_root_dir.get_parameter_value().string_value, 'coppeliaSim.sh'),
                '-h',
                '-s',
                '0',
                self.coppelia_scene_path.get_parameter_value().string_value,
                ])
        else:
            subprocess.run([
                '/bin/bash', 
                os.path.join(self.coppelia_root_dir.get_parameter_value().string_value, 'coppeliaSim.sh'),
                '-s',
                '0',
                self.coppelia_scene_path.get_parameter_value().string_value,
                ])


def main():
    rclpy.init()
    coppelia_node = CoppeliaNode()
    coppelia_node.run()
    try:
        rclpy.spin(coppelia_node)
    except:
        pass
    coppelia_node.destroy_node()
    rclpy.shutdown()
        