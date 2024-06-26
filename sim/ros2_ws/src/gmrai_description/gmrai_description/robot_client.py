from dotenv import load_dotenv
import json
import os
from enum import Enum
import argparse
from google.cloud import storage
from gmrai_description.ply2jpg import run_conversion

import trimesh
import os

from ament_index_python import get_package_share_directory

def convert_obj_to_glb(file_path):
    # Load the OBJ file
    mesh = trimesh.load(file_path)

    # Change the file extension to .glb
    glb_file_path = os.path.splitext(file_path)[0] + '.glb'

    # Export the mesh to GLB format
    mesh.export(glb_file_path, file_type='glb')

    print(f"Converted {file_path} to {glb_file_path}")


def upload_to_gcs(file_path, destination_blob_name):
    # Initialize a storage client
    storage_client = storage.Client()
    bucket = storage_client.bucket(os.environ.get("BUCKET_NAME"))
    blob = bucket.blob(destination_blob_name)

    # Upload the file
    blob.upload_from_filename(file_path)
    blob.make_public()

    print(f"File {file_path} uploaded to {destination_blob_name}.")


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
                    self.robot_manager.get_logger().info(f"Robot is online.")
                    self.connection = CON_STATUS.ONLINE
                    # Reanudate status
                    self.reanudate_status()
                                
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

    def reanudate_status(self):
        try:
            response = requests.post(f"{self.server_url}/active_job", json={'code': self.code})
            data = response.json()
            if data:
                self.parse_job_status(data)
        except requests.RequestException as e:
            self.robot_manager.get_logger().info(f"Error: {e}")
        return None

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
            if job_status == State.WORKING and self.active_job != None:
                self.robot_manager.get_logger().info("Cancelling current job...")
                self.cancel_task()
            
            self.new_job()            
        elif job_status == j_status.START_JOB:
            job_data = data.get('job_data')
            if not job_data:
                self.robot_manager.get_logger().info(f"Error: Job data was not given, cancelling...")
                return
            self.robot_state = State.WORKING

            if job_status == State.WORKING and self.active_job != None:
                if self.active_job['id'] == job_data['id']:
                    self.robot_manager.get_logger().info(f"Already doing the job")
                    return

                self.robot_manager.get_logger().info(f"Cancelling current job...")
                self.cancel_task()
            else:
                job_status = State.WORKING

            self.do_task(list(job_data['area'].values()), int(job_data['cutting_height']))
            self.active_job = job_data
            return
        elif job_status == j_status.CANCEL_JOB:
            if self.active_job is not None:
                self.robot_manager.get_logger().info(f"Cancelling...")
                self.cancel_task()
            self.send_finished()
            self.robot_state = State.IDLE
            return
        elif job_status == j_status.UPDATE_JOB:
            # Enviar datos del job ?????????
            return

################# JOBS #################

    def do_task(self, area, height):
        self.robot_manager.get_logger().info(f"Setting cutting height...")
        self.robot_manager.publish_grass_height(height)
        self.robot_manager.publish_reconstruction_switch(True)
        self.robot_manager.startup()
        self.robot_manager.get_logger().info(f"Planning the task...")
        self.robot_manager.start_navigation(area=area)
        self.robot_manager.get_logger().info(f"Plan was sent to ROS2")
        return

    def cancel_task(self):
        self.robot_manager.cancel_navigation()
        self.send_finished()
        self.robot_state = State.IDLE
        return
    
    def new_job(self):
        file_noext = self.robot_manager.get_reconstruction()
        obj_file = file_noext + ".obj"
        glb_file = file_noext + ".glb"
        ply_file = file_noext + ".ply"
        jpg_file = file_noext + ".jpg"

        convert_obj_to_glb(obj_file)

        self.robot_manager.get_logger().info(f"Making the top image")
        run_conversion(ply_file, jpg_file)

        self.robot_manager.get_logger().info(f"Sending data...")
        upload_to_gcs(glb_file, f"{self.code}_gmr.glb")
        upload_to_gcs(jpg_file, f"{self.code}_gmr.jpg")
        self.send_finished()
    
    def send_finished(self):
        try:
            data = {'code': self.code}
            response = requests.post(f"{self.server_url}/job_finished", json=data)
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

            elif self.robot_state == State.WORKING:
                if not self.robot_manager.is_navigation_finished():
                    self.robot_manager.get_logger().info(f"Feedback: {self.robot_manager.get_feedback()}")
                else:
                    self.robot_manager.get_logger().info(f"Set next goal")
                    is_still_working = self.robot_manager.start_navigation()
                    if not is_still_working:
                        self.robot_manager.get_logger().info(f"Finished job")
                        self.cancel_task()
                        
            else: # Unemployed, get a job
                self.check_job_updates()
