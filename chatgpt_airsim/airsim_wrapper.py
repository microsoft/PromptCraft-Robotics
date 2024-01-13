import base64
import json
import math
import random
import threading
import time

import airsim
import numpy as np
import openai
import requests
from airsim import Client, ImageRequest, ImageType

objects_dict = {
    "turbine1": "BP_Wind_Turbines_C_1",
    "turbine2": "StaticMeshActor_2",
    "solarpanels": "StaticMeshActor_146",
    "crowd": "StaticMeshActor_6",
    "car": "StaticMeshActor_10",
    "tower1": "SM_Electric_trellis_179",
    "tower2": "SM_Electric_trellis_7",
    "tower3": "SM_Electric_trellis_8",
}


class AirSimWrapper:
    def __init__(self):
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        self.stop_thread = False
        self.flutter_thread = None

    def takeoff(self):
        self.client.takeoffAsync().join()

    def land(self):
        self.client.landAsync().join()

    def get_drone_position(self):
        pose = self.client.simGetVehiclePose()
        return [pose.position.x_val, pose.position.y_val, pose.position.z_val]

    def fly_to(self, point):
        if point[2] > 0:
            self.client.moveToPositionAsync(point[0], point[1], -point[2], 5).join()
        else:
            self.client.moveToPositionAsync(point[0], point[1], point[2], 5).join()

    def fly_path(self, points):
        airsim_points = []
        for point in points:
            if point[2] > 0:
                airsim_points.append(airsim.Vector3r(point[0], point[1], -point[2]))
            else:
                airsim_points.append(airsim.Vector3r(point[0], point[1], point[2]))
        self.client.moveOnPathAsync(
            airsim_points,
            5,
            120,
            airsim.DrivetrainType.ForwardOnly,
            airsim.YawMode(False, 0),
            20,
            1,
        ).join()

    def set_yaw(self, yaw):
        self.client.rotateToYawAsync(yaw, 5).join()

    def get_yaw(self):
        orientation_quat = self.client.simGetVehiclePose().orientation
        yaw = airsim.to_eularian_angles(orientation_quat)[2]
        return yaw

    def get_position(self, object_name):
        query_string = objects_dict[object_name] + ".*"
        object_names_ue = []
        while len(object_names_ue) == 0:
            object_names_ue = self.client.simListSceneObjects(query_string)
        pose = self.client.simGetObjectPose(object_names_ue[0])
        return [pose.position.x_val, pose.position.y_val, pose.position.z_val]

    @staticmethod
    def is_within_boundary(start_pos, current_pos, limit_radius):
        """Check if the drone is within the spherical boundary"""
        distance = math.sqrt(
            (current_pos.x_val - start_pos.x_val) ** 2
            + (current_pos.y_val - start_pos.y_val) ** 2
            + (current_pos.z_val - start_pos.z_val) ** 2
        )
        return distance <= limit_radius

    def flutter(self, speed=5, change_interval=1, limit_radius=10):
        """Simulate Brownian motion /fluttering with the drone"""
        # Takeoff and get initial position
        self.client.takeoffAsync().join()
        start_position = self.client.simGetVehiclePose().position

        while not self.stop_thread:
            # Propose a random direction
            pitch = random.uniform(-1, 1)  # Forward/backward
            roll = random.uniform(-1, 1)  # Left/right
            yaw = random.uniform(-1, 1)  # Rotate

            # Move the drone in the proposed direction
            self.client.moveByRollPitchYawrateThrottleAsync(
                roll, pitch, yaw, 0.5, change_interval
            ).join()

            # Get the current position
            current_position = self.client.simGetVehiclePose().position

            # Check if the drone is within the boundary
            if not self.is_within_boundary(
                start_position, current_position, limit_radius
            ):
                # If outside the boundary, adjust to a new random direction
                self.client.moveToPositionAsync(
                    start_position.x_val,
                    start_position.y_val,
                    start_position.z_val,
                    speed,
                ).join()

            # Wait for the next change
            time.sleep(change_interval)

    def start_fluttering(self, speed=5, change_interval=1, limit_radius=10):
        self.stop_thread = False
        self.flutter_thread = threading.Thread(
            target=self.flutter, args=(speed, change_interval, limit_radius)
        )
        self.flutter_thread.start()

    def stop_fluttering(self):
        self.stop_thread = True
        if self.flutter_thread is not None:
            self.flutter_thread.join()

    def take_photo(client: Client, vehicle_name: str):
        request = ImageRequest("0", ImageType.scene, False, False)
        response = client.simGetImages([request], vehicle_name)

        if response:
            image_response = response[0]

            image_response = base64.b64encode(image_response).decode("utf-8")

            return image_response.image_data_uint8

    def analyze_with_vision_model(image_data):
        # Load API key from config.json
        with open("config.json") as f:
            data = json.load(f)
            api_key = data["API_key"]

        # Convert image data to base64
        base64_image = base64.b64encode(image_data).decode("utf-8")

        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {api_key}",
        }

        payload = {
            "model": "gpt-4-vision-preview",
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": "How many people are in this image?"},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{base64_image}"
                            },
                        },
                    ],
                }
            ],
            "max_tokens": 2000,
        }

        response = requests.post(
            "https://api.openai.com/v1/chat/completions", headers=headers, json=payload
        )

        # Return the response
        return response.json()
