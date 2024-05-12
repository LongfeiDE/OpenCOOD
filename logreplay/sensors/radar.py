import weakref

import carla
import os
import open3d as o3d
import numpy as np
from logreplay.sensors.base_sensor import BaseSensor

class Radar(BaseSensor):

    def __init__(self, agent_id, vehicle, world, config, global_position):
        super().__init__(agent_id, vehicle, world, config, global_position)

        if vehicle is not None:
            world = vehicle.get_world()

        self.vehicle = vehicle
        self.agent_id = agent_id
        self.name = 'radar'

        blueprint = world.get_blueprint_library().find('sensor.other.radar')
        blueprint.set_attribute('horizontal_fov', str(config['horizontal_fov']))
        blueprint.set_attribute('vertical_fov', str(config['vertical_fov']))
        blueprint.set_attribute('points_per_second', str(config['points_per_second']))
        blueprint.set_attribute('range', str(config['range']))

        if vehicle is None:
            spawn_point = self.spawn_point_estimation(None, global_position)
            self.sensor = world.spawn_actor(blueprint, spawn_point)
        else:
            self.relative_position = config['relative_pose']
            self.relative_position_id = ['front', 'right', 'left', 'back']
            spawn_point = self.spawn_point_estimation(self.relative_position, None)
            self.sensor = world.spawn_actor(blueprint, spawn_point, attach_to=vehicle)
            self.name += str(self.relative_position)

        # radar data
        self.thresh = config['thresh'] # not used, could think about some methods
        self.data = None
        self.timestamp = None
        self.frame = 0
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: Radar._on_data_event(weak_self, event))

    @staticmethod
    def _on_data_event(weak_self, event):
        """Radar  method"""
        self = weak_self()
        if not self:
            return
        all_points = []
        # retrieve the raw radar data and reshape to (N, 4)
        radar_data = np.copy(np.frombuffer(event.raw_data, dtype=np.dtype('f4')))
        # (depth, velocity, azimuth, altitude)
        for detect in event:
            # for all points
            all_points.append([detect.depth, detect.velocity, detect.azimuth, detect.altitude])
        data = np.asarray(all_points)

        self.data = data
        self.frame = event.frame
        self.timestamp = event.timestamp

    @staticmethod
    def spawn_point_estimation(relative_position, global_position):

        pitch = 0
        carla_location = carla.Location(x=0, y=0, z=0)

        if global_position is not None:
            carla_location = carla.Location(
                x=global_position[0],
                y=global_position[1],
                z=global_position[2])

            carla_rotation = carla.Rotation(pitch=global_position[3], yaw=global_position[4], roll=global_position[5])

        else:

            if relative_position == 'front':
                carla_location = carla.Location(x=carla_location.x + 2.5,
                                                y=carla_location.y,
                                                z=carla_location.z + 1.0)
                yaw = 0

            elif relative_position == 'right':
                carla_location = carla.Location(x=carla_location.x + 0.0,
                                                y=carla_location.y + 0.3,
                                                z=carla_location.z + 1.8)
                yaw = 100

            elif relative_position == 'left':
                carla_location = carla.Location(x=carla_location.x + 0.0,
                                                y=carla_location.y - 0.3,
                                                z=carla_location.z + 1.8)
                yaw = -100
            else:
                carla_location = carla.Location(x=carla_location.x - 2.0,
                                                y=carla_location.y,
                                                z=carla_location.z + 1.5)
                yaw = 180

            carla_rotation = carla.Rotation(roll=0, yaw=yaw, pitch=pitch)

        spawn_point = carla.Transform(carla_location, carla_rotation)

        return spawn_point

    def data_dump(self, output_root, cur_timestamp):

        while not hasattr(self, 'data') or self.data is None:
            continue

        radar_data = self.data



        # write to npy file
        if self.vehicle is None:
            npy_name = f'{cur_timestamp}.npy'
        else:
            pose_id = self.relative_position_id.index(self.relative_position)
            npy_name = f'{cur_timestamp}_radar{pose_id}.npy'

        np.save(os.path.join(output_root, npy_name), radar_data)
