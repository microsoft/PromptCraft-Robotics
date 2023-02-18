import airsim


class AirSimWrapper:
    def __init__(self):
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

    def takeoff(self):
        self.client.takeoffAsync().join()

    def get_drone_position(self):
        pose = self.client.simGetVehiclePose()
        return [pose.position.x_val, pose.position.y_val, pose.position.z_val]

    def fly_to(self, x, y, z):
        if z > 0:
            self.client.moveToPositionAsync(x, y, -z, 5).join()
        else:
            self.client.moveToPositionAsync(x, y, z, 5).join()

    def set_yaw(self, yaw):
        self.client.rotateToYawAsync(yaw, 5).join()

    def get_yaw(self):
        orientation_quat = self.client.simGetVehiclePose().orientation
        yaw = airsim.to_eularian_angles(orientation_quat)[2]
        return yaw

    def get_position(self, object_name):
        query_string = object_name + ".*"
        object_name_ue = self.client.simListSceneObjects(query_string)[0]
        pose = self.client.simGetObjectPose(object_name_ue)
        return [pose.position.x_val, pose.position.y_val, pose.position.z_val]
