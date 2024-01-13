import base64
import json
import math
import os
import random
import threading
import time

import airsim
import cv2
import numpy as np
import openai
import requests

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

    def generate_circular_path(center, radius, height, segments=12):
        path = []
        for i in range(segments):
            angle = 2 * math.pi * i / segments
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)
            z = height
            path.append(x, y, z)
        return path

    def take_photo(self, filename):
        responses = self.client.simGetImages(
            [airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)]
        )
        response = responses[0]

        # get numpy array
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)

        # reshape array to 3 channel image array H X W X 3
        img_rgb = img1d.reshape(response.height, response.width, 3)

        # # original image is flipped vertically
        # img_rgb = np.flipud(img_rgb)

        # write to png
        filename = os.path.normpath(filename + ".png")
        cv2.imwrite(filename, img_rgb)

        # encode image to base64 string
        with open(filename, "rb") as image_file:
            base64_image = base64.b64encode(image_file.read()).decode("utf-8")

        return base64_image

    def analyze_with_vision_model(self, image_data):
        # Load API key from config.json
        with open("config.json") as f:
            data = json.load(f)
            api_key = data["OPENAI_API_KEY"]

        if isinstance(image_data, str):
            image_data = image_data.encode()

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

        # Google Vision API: https://cloud.google.com/vision/docs/object-localizer
        # path = "path to image"
        # client = vision.ImageAnnotatorClient()

        # with open(path, "rb") as image_file:
        #     content = image_file.read()
        # image = vision.Image(content=content)

        # objects = client.object_localization(image=image).localized_object_annotations

    def query_language_model(prompt):
        with open("config.json", "r") as f:
            config = json.load(f)
        openai.api_key = config["OPENAI_API_KEY"]
        chat_history = [
            {
                "role": "user",
                "content": prompt,
            }
        ]
        completion = openai.ChatCompletion.create(
            model="gpt-3.5-turbo", messages=chat_history, temperature=0
        )
        return completion.choices[0].message.content

    # Complex commands
    def count(self, object_name):
        filename = "count" + object_name + ".jpg"
        image_data = self.take_photo(filename)
        vision_outputs = self.analyze_with_vision_model(image_data)
        # Naive: converts vision model json output to string, append to count prompt
        prompt = (
            "\n Based on this json output, count the number of instances of "
            + object_name
            + " in the scene. Return a single number"
        )
        return self.query_language_model(str(vision_outputs) + prompt)

    def search(self, object_name, radius):
        # code motion
        self.fly_to(self.get_position(object_name))
        # fly in a circle
        circular_path = self.generate_circular_path(
            self.get_position(object_name)[:2],
            radius,
            self.get_position(object_name)[2],
        )
        vision_outputs = []
        for point in circular_path:
            self.fly_to(point)
            image_data = self.take_photo(str(point))
            vision_output = self.analyze_with_vision_model(image_data)
            vision_outputs.append(vision_output)
        prompt = (
            "\n Based on these json outputs, is "
            + object_name
            + "present in the scene? Return TRUE or FALSE."
        )
        return self.query_language_model(str(vision_outputs) + prompt)

    def get_latitude_longitude(self, object_name):
        self.fly_to(self.get_position(object_name))
        return (
            self.get_position(object_name)[0],
            self.get_drone_position(object_name)[1],
        )
