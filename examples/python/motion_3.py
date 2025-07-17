import math
import PyKDL as kdl
from abc import ABC

class Trajectory(ABC):
    def position(self, time):
        pass

    def velocity(self, time):
        pass

    def acceleration(self, time):
        pass

    def duration(self):
        pass

class TrajectorySegment(Trajectory):
    def __init__(self, path, profile, duration):
        self._path = path
        self._profile = profile
        self._profile.set_profile_duration(0, path.path_length(), duration)
        # print(f"DEBUG: Initializing TrajectorySegment with duration: {duration}")

    def position(self, time):
        return self._path.position(self._profile.position(time))

    def velocity(self, time):
        return self._path.velocity(self._profile.position(time), self._profile.velocity(time))

    def acceleration(self, time):
        return self._path.acceleration(self._profile.position(time), self._profile.velocity(time), self._profile.acceleration(time))

    def duration(self):
        return self._profile.duration()

class TrajectoryComposite(Trajectory):
    def __init__(self):
        self._duration = 0.0
        self._trajectories = []
        self._end_times = []

    def add_segment(self, segment):
        self._trajectories.append(segment)
        self._duration += segment.duration()
        self._end_times.append(self._duration)

    def position(self, time):
        if time < 0:
            return self._trajectories[0].position(0)

        previous_time = 0.0

        for i, traj in enumerate(self._trajectories):
            if time < self._end_times[i]:
               #  print(f"DEBUG: Time {time} is less than end time {self._end_times[i]}. Selected segment {i}.")
                return traj.position(time - previous_time)

            previous_time = self._end_times[i]

        last_traj = self._trajectories[-1]
       # print(f"DEBUG: Time {time} exceeds total duration. Returning position at end time: {last_traj.duration()}")
        return last_traj.position(last_traj.duration())

    def velocity(self, time):
        if time < 0:
            return self._trajectories[0].velocity(0)

        previous_time = 0.0

        for i, traj in enumerate(self._trajectories):
            if time < self._end_times[i]:
                return traj.velocity(time - previous_time)

            previous_time = self._end_times[i]

        last_traj = self._trajectories[-1]
        return last_traj.velocity(last_traj.duration())

    def acceleration(self, time):
        if time < 0:
            return self._trajectories[0].acceleration(0)

        previous_time = 0.0

        for i, traj in enumerate(self._trajectories):
            if time < self._end_times[i]:
                return traj.acceleration(time - previous_time)

            previous_time = self._end_times[i]

        last_traj = self._trajectories[-1]
        return last_traj.acceleration(last_traj.duration())

    def duration(self):
        return self._duration

class Path(ABC):
    def position(self, s):
        pass

    def velocity(self, s, sd):
        pass

    def acceleration(self, s, sd, sdd):
        pass

    def path_length(self):
        pass

class PathLine(Path):
    def __init__(self, H_base_start, H_base_end):
        self._p_base_start = H_base_start.p
        self._p_base_end = H_base_end.p
        self._p_start_end = self._p_base_end - self._p_base_start
        dist = self._p_start_end.Normalize()
        self._pathlength = dist

    def position(self, s):
        return kdl.Frame(self._p_base_start + self._p_start_end * s)

    def velocity(self, s, sd):
        return kdl.Twist(self._p_start_end * sd, kdl.Vector.Zero())

    def acceleration(self, s, sd, sdd):
        return kdl.Twist(self._p_start_end * sdd, kdl.Vector.Zero())

    def path_length(self):
        return self._pathlength

class PathCircle(Path):
    def __init__(self, H_base_start, p_base_center, alpha):
        self._H_base_start = H_base_start
        self._H_base_center = kdl.Frame.Identity()
        self._H_base_center.p = p_base_center
        self._alpha = alpha
        
        # Vector desde el centro al punto de inicio
        self._radius_vector = H_base_start.p - self._H_base_center.p
        self._radius = self._radius_vector.Norm()
        
        # Path length es el arco que recorrerá
        self._pathlength = abs(alpha * self._radius)
        self._direction = 1 if alpha > 0 else -1

    def position(self, s):
        angle = self._direction * s / self._radius
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)

        # Rotación en el plano XY alrededor del centro
        p = self._H_base_center * kdl.Vector(self._radius_vector.x() * cos_angle - self._radius_vector.y() * sin_angle,self._radius_vector.x() * sin_angle + self._radius_vector.y() * cos_angle,self._radius_vector.z())
        
        return kdl.Frame(p)

    def velocity(self, s, sd):
        angle = self._direction * s / self._radius
        v = sd / self._radius
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        
        return kdl.Twist(kdl.Vector(-self._radius * sin_angle * v,self._radius * cos_angle * v, 0 ),kdl.Vector.Zero() )

    def acceleration(self, s, sd, sdd):
        angle = self._direction * s / self._radius
        v = sd / self._radius
        a = sdd / self._radius
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)

        return kdl.Twist(kdl.Vector(-self._radius * cos_angle * v * v - self._radius * sin_angle * a, -self._radius * sin_angle * v * v + self._radius * cos_angle * a,  0), kdl.Vector.Zero()  )

    def path_length(self):
        return self._pathlength


class VelocityProfile(ABC):
    def position(self, time):
        pass

    def velocity(self, time):
        pass

    def acceleration(self, time):
        pass

    def duration(self):
        pass

    def set_profile(self, pos1, pos2):
        pass

    def set_profile_duration(self, pos1, pos2, duration):
        pass

class VelocityProfileRectangular(VelocityProfile):
    def __init__(self, maxvel):
        self._maxvel = maxvel
        self._duration = 0
        self._position = 0
        self._velocity = 0

    def position(self, time):
        if time < 0:
            return self._position
        elif time > self._duration:
            return self._position + self._velocity * self._duration
        else:
            return self._position + self._velocity * time

    def velocity(self, time):
        if time < 0:
            return 0
        elif time > self._duration:
            return 0
        else:
            return self._velocity

    def acceleration(self, time):
        return 0

    def duration(self):
        return self._duration

    def _set_max_velocity(self, maxvel):
        self._maxvel = maxvel

    def set_profile(self, pos1, pos2):
        diff = pos2 - pos1

        if diff != 0:
            self._velocity = self._maxvel if diff > 0 else -self._maxvel
            self._position = pos1
            self._duration = diff / self._velocity
        else:
            self._velocity = 0
            self._position = pos1
            self._duration = 0
       # print(f"DEBUG: Set profile - pos1: {pos1}, pos2: {pos2}, velocity: {self._velocity}, duration: {self._duration}")

    def set_profile_duration(self, pos1, pos2, duration):
        diff = pos2 - pos1

        if diff != 0:
            self._velocity = diff / duration

            if self._velocity > self._maxvel or duration == 0:
                self._velocity = self._maxvel

            self._position = pos1
            self._duration = diff / self._velocity
        else:
            self._velocity = 0
            self._position = pos1
            self._duration = 0
        # print(f"DEBUG: Set profile duration - pos1: {pos1}, pos2: {pos2}, duration: {duration}, velocity: {self._velocity}, final duration: {self._duration}")