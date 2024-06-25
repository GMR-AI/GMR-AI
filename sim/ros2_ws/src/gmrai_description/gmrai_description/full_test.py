import trimesh
import os
import time

from gmrai_description.ply2jpg import run_conversion

def convert_obj_to_glb(file_path):
    # Load the OBJ file
    mesh = trimesh.load(file_path)

    # Change the file extension to .glb
    glb_file_path = os.path.splitext(file_path)[0] + '.glb'

    # Export the mesh to GLB format
    mesh.export(glb_file_path, file_type='glb')

    print(f"Converted {file_path} to {glb_file_path}")

def new_job(robot_manager):
    file_noext = robot_manager.get_reconstruction()
    obj_file = file_noext + ".obj"
    glb_file = file_noext + ".glb"
    ply_file = file_noext + ".ply"
    jpg_file = file_noext + ".jpg"

    convert_obj_to_glb(obj_file)

    robot_manager.get_logger().info(f"Making the top image")
    run_conversion(ply_file, jpg_file)

    robot_manager.get_logger().info(f"Sending data...")
    #upload_to_gcs(glb_file, f"{self.code}_gmr.glb")
    #upload_to_gcs(jpg_file, f"{self.code}_gmr.jpg")
    #send_finished()

def do_task(robot_manager, area):
    robot_manager.publish_reconstruction_switch(True)
    robot_manager.startup()
    robot_manager.get_logger().info(f"Planning the task...")
    robot_manager.start_navigation(area)
    robot_manager.get_logger().info(f"Plan was sent to ROS2")

def run(robot_manager):
    while True:
        time.sleep(3)
        if not robot_manager.is_navigation_finished(): # Also it can check if robot is idle or working on some job
            robot_manager.get_logger().info(f"Feedback: {robot_manager.get_feedback()}")
            time.sleep(1)
        else:
            robot_manager.get_logger().info(f"Set next goal")
            is_still_working = robot_manager.start_navigation()
            if not is_still_working:
                robot_manager.get_logger().info(f"Returned to home position")
                return
        
        # ping()
        # robot_manager.get_logger().info(f"State: {robot_state}")
        # No online, no new activities
        # if connection == CON_STATUS.OFFLINE:
        #     continue
        # elif robot_state == State.REQUESTING: # Check for the request
        #     check_request()
        # else: # Unemployed, get a job
        #     check_job_updates()

def full_test(robot_manager):
    # Add new job
    new_job(robot_manager)

    # Simulate wait for user to decide
    time.sleep(5)

    # Start new job
    area = [[44, 53], [185, 57], [178, 197], [43, 196]]
    do_task(robot_manager, area)

    # Keep doing the job until finished
    run(robot_manager)
    