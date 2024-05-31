from dotenv import load_dotenv
import json
import os
from enum import Enum
import argparse

from gmrai_description.ply2jpg import run_conversion

import trimesh
import os

from ament_index_python import get_package_share_directory

def convert_obj_to_glb(obj_path, glb_path):
    # Load the OBJ file
    mesh = trimesh.load(obj_path)

    # Export the mesh to GLB format
    mesh.export(glb_path, file_type='glb')

    print(f"Converted {obj_path} to {glb_path}")


# Ordenes que el usuario puede enviar directamente al robot (estas se resetean a NONE una vez el robot reciba la orden)
class j_status(Enum):
    NONE = 0 # No updates
    NEW_JOB = 1 # User requested 3D model and top image
    START_JOB = 2 # User has uploaded the new job
    UPDATE_JOB = 3 # User recalls any update in the actual job?
    CANCEL_JOB = 4 # User has canceled the job


class State(Enum):
    IDLE = 0
    REQUESTING = 1
    WORKING = 2
    FINISHING = 3

class CON_STATUS(Enum):
    OFFLINE = 0
    ONLINE = 1

class Job:
    def __init__(self, server_url, data, debug=False):
        self.server_url = server_url
        self.code = data['matricula']
        self.model = data['model_id']
        self.active_job = None
        self.robot_state = State.IDLE
        self.connection = CON_STATUS.OFFLINE
        self.debug = debug

import requests
import time

class RobotClient:

################# ATRIBUTES ######################


    def __init__(self, server_url, data, robot_manager, debug=False):
        self.server_url = server_url
        self.code = data['matricula']
        self.model = data['model_id']
        self.active_job = None
        self.robot_state = State.IDLE
        self.connection = CON_STATUS.OFFLINE
        self.debug = debug
        self.robot_manager = robot_manager

################# HEALTH CHECKS #################

    def log_message(self, response):
            # Check for any logs
            data = response.json()
            log = data.get("message")

            if log: 
                self.robot_manager.get_logger().info(f"Server says: {log}")

    def ping(self):
        try:
            response = requests.post(f"{self.server_url}/ping", json={'code': self.code})
            if response.status_code == 200:
                self.log_message(response)
                if self.connection == CON_STATUS.OFFLINE:
                    self.robot_manager.get_logger().info(f"Robot is oglbnline.")
                    self.connection = CON_STATUS.ONLINE
                                
            elif response.status_code == 201:
                if self.robot_state != State.REQUESTING:
                    self.robot_manager.get_logger().info(f"Requesting...")
                    self.send_request()
            else:
                self.robot_manager.get_logger().info(f"Failed to go online: {response.status_code}")
                self.connection = CON_STATUS.OFFLINE

        except requests.RequestException as e:
            self.connection = CON_STATUS.OFFLINE
            self.robot_manager.get_logger().info(f"Error in connection, maybe server is offline")
            if self.debug:
                self.robot_manager.get_logger().info(f"Error: {e}")

################# REQUESTING #################

    def send_request(self):
        try:
            response = requests.post(f"{self.server_url}/new_request", json={'code': self.code, 'model': self.model})
            if response.status_code == 201:
                # If everything is correct return to default state
                self.robot_state = State.REQUESTING
                self.log_message(response)
        except requests.RequestException as e:
            self.robot_manager.get_logger().info(f"Error: {e}")

    def check_request(self):
        try:
            response = requests.post(f"{self.server_url}/check_request", json={'code': self.code})
            if response.status_code == 200:
                # If everything is correct return to default state
                self.robot_state = State.IDLE
        except requests.RequestException as e:
            self.robot_manager.get_logger().info(f"Error: {e}")

################# IDLE #################

    def check_job_updates(self):
        try:
            response = requests.post(f"{self.server_url}/active_job", json={'code': self.code})
            if response.status_code == 200:
                data = response.json()
                self.parse_job_status(data)
            else:
                self.robot_manager.get_logger().info(f"Failed to retrieve active job: {response.status_code}")
        except requests.RequestException as e:
            self.robot_manager.get_logger().info(f"Error: {e}")
        return None
    
    def parse_job_status(self, data):
        job_status_str =  data.get('job_status')
        job_status = j_status[job_status_str]

        if job_status == j_status.NONE:
            return
        elif job_status == j_status.NEW_JOB:
            # Funcion de reconstruccion
            model_path = self.robot_manager.get_reconstruction()
            obj_path = model_path + '.obj'
            ply_path = model_path + '.ply'
            glb_path = model_path + '.glb'
            jpg_path = model_path + '.jpg'

            self.robot_manager.get_logger().info(f'Converting from obj to glb...')
            convert_obj_to_glb(obj_path, glb_path)
            self.robot_manager.get_logger().info(f'Converted from obj to glb!')

            self.robot_manager.get_logger().info(f'Making top-view from the ply...')
            run_conversion(ply_path, jpg_path)
            self.robot_manager.get_logger().info(f'Made top-view from the ply...')

            # self.robot_manager.get_logger().info(f'Sending glb...')
            # self.upload_file(os.path.splitext(model_path)[0] + '.glb')
            # self.robot_manager.get_logger().info(f'Sent glb!')

            self.robot_manager.get_logger().info(f'Sending jpg...')
            self.upload_file(jpg_path)
            self.robot_manager.get_logger().info(f'Sent jpg!')

            self.send_finished()
            
        elif job_status == j_status.START_JOB:
            job_data = data.get('job_data')
            if not job_data:
                self.robot_manager.get_logger().info(f"Error: Job data was not given, cancelling...")
                return

            if job_status == State.WORKING and self.active_job != None:
                if self.active_job.id == job_data['id']:
                    self.robot_manager.get_logger().info(f"Already doing the job")
                    return

                self.robot_manager.get_logger().info(f"Cancelling current job...")
                self.cancel_task()
            else:
                job_status = State.WORKING

            self.robot_manager.get_logger().info(f"Starting job...")
            test_position = [3.0, 2.0, 0.0]
            self.robot_manager.start_navigation(test_position)
            return
        elif job_status == j_status.CANCEL_JOB:
            self.robot_manager.cancel_navigation()
            # pos = [ , , 0.0]
            # self.robot_manager.navigate(pos)
            return
        elif job_status == j_status.UPDATE_JOB:
            # Enviar datos del job
            
            return

################# JOBS #################

    def do_task(self):
        return

    def cancel_task(self):
        return
    
    def upload_file(self, file_path):
        try:
            with open(file_path, 'rb') as file:
                files = {'file': file}
                data = {'code': self.code}
                response = requests.post(f"{self.server_url}/upload_file", files=files, json=data)
                if response.status_code == 200:
                    self.robot_manager.get_logger().info(f"File uploaded successfully.")
                else:
                    self.robot_manager.get_logger().info(f"Failed to upload file: {response.status_code} {response.json()}")
        except Exception as e:
            self.robot_manager.get_logger().info(f"Error: {e}")
    
    def send_finished(self):
        try:
            data = {'code': self.code}
            response = requests.post(f"{self.server_url}/job_finished", json={'code': self.code})
            if response.status_code == 200:
                self.robot_manager.get_logger().info(f"Task finished successfully.")
            else:
                self.robot_manager.get_logger().info(f"Failed to notify the finished task: {response.status_code} {response.json()}")
        except Exception as e:
            self.robot_manager.get_logger().info(f"Error: {e}")


################# MAIN LOOP #################

    def run(self):
        while True:
            time.sleep(3)
            self.ping()
            self.robot_manager.get_logger().info(f"State: {self.robot_state}")
            # No online, no new activities
            if self.connection == CON_STATUS.OFFLINE:
                continue
            elif self.robot_state == State.REQUESTING: # Check for the request
                self.check_request()
            else: # Unemployed, get a job
                self.check_job_updates()
