from dataclasses import dataclass
import queue
import numpy as np
import matplotlib.pyplot as plt
import time

from .minsnap import minimum_snap_traj_p2p, get_traj


def map_thrust(thrust):
    """Remap thrust from 0 to 50000"""
    return int(0 + (thrust) * (49999 - 0) / (100))


@dataclass
class CFSetpoint:
    priority: int = None
    setpoint: tuple = None
    setpointType: str = None
    timestamp: float = None

    # def __eq__(self, other: object) -> bool:
    #     return self.priority == other.priority

    def __lt__(self, other: object) -> bool:
        if self.priority == other.priority:
            return self.timestamp < other.timestamp
        else:
            return self.priority < other.priority

    def __gt__(self, other: object) -> bool:
        if self.priority == other.priority:
            return self.timestamp > other.timestamp
        else:
            return self.priority > other.priority


@dataclass
class PositionSetpoint(CFSetpoint):
    def __init__(
        self, x: float, y: float, z: float, yaw: float, timeTo: float, priority: int = 0
    ) -> None:
        """Position setpoint to send to drone

        Args:
            x (float): x position in metres
            y (float): y position in metres
            z (float): z position in metres
            yaw (float): yaw in degrees
            timeTo (float): time to fly to position in seconds
            priority (int): priority in queue (highest goes first)
        """
        super().__init__()
        # PriorityQueue prioritises lowest number, highest is more logical
        self.priority = -priority
        self.setpoint = (x, y, z, np.radians(yaw))
        self.timeTo = timeTo
        self.setpointType = "position"
        self.v = [0, 0, 0, 0]
        self.a = [0, 0, 0, 0]

    def __repr__(self) -> str:
        return f"x = {self.setpoint[0]}m | y = {self.setpoint[1]}m | z = {self.setpoint[2]}m | yaw = {self.setpoint[3]}rad"

@dataclass
class RelativePositionSetpoint(CFSetpoint):
    def __init__(
        self, x: float, y: float, z: float, yaw: float, timeTo: float, priority: int = 0, currentYAW:float = 0
    ) -> None:
        """Position setpoint to send to drone, but RELATIVE. Drone body reference instead of inertial

        Args:
            x (float): x position in metres
            y (float): y position in metres
            z (float): z position in metres
            yaw (float): yaw in degrees
            timeTo (float): time to fly to position in seconds
            priority (int): priority in queue (highest goes first)
        """
        super().__init__()
        # PriorityQueue prioritises lowest number, highest is more logical
        xyz = np.array([x,y,z])
        if x+y+z > 0:
            print(xyz, "cyaw=", currentYAW)
        # currentYAW = -np.radians(currentYAW)
        currentYAW = -currentYAW

        transform = np.array([[np.cos(currentYAW), np.sin(currentYAW), 0],
                              [-np.sin(currentYAW), np.cos(currentYAW), 0],
                              [0,0,1]])

        xyznew = list(transform @ xyz)
        if x+y+z > 0:
            print(xyznew)
        
        self.priority = -priority
        self.setpoint = (xyznew[0], xyznew[1], xyznew[2], np.radians(yaw))
        # print(self.setpoint)
        self.timeTo = timeTo
        self.setpointType = "relative"
        self.v = [0, 0, 0, 0]
        self.a = [0, 0, 0, 0]

    def __repr__(self) -> str:
        return f"x = {self.setpoint[0]}m | y = {self.setpoint[1]}m | z = {self.setpoint[2]}m | yaw = {self.setpoint[3]}rad"


@dataclass
class AnglesSetpoint(CFSetpoint):
    def __init__(
        self,
        roll: float,
        pitch: float,
        yawrate: float,
        thrust: float,
        holdTime: float,
        priority: int = 0,
    ) -> None:
        """Angles (default) setpoint to send to drone

        Args:
            roll (float): absolute roll (right positive)
            pitch (float): absolute pitch (down positive)
            yawrate (float): yaw rate
            thrust (float): thrust (float between 0 and 100)
            holdTime (float): time in seconds to hold setpoint
            priority (int): priority in queue (highest goes first)
        """
        super().__init__()
        # PriorityQueue prioritises lowest number, highest is more logical
        self.priority = -priority
        self.holdTime = holdTime
        self.setpoint = (roll, pitch, yawrate, map_thrust(thrust))
        self.setpointType = "angles"

    def __repr__(self) -> str:
        return f"roll = {self.setpoint[0]}deg | pitch = {self.setpoint[1]}deg | yawrate = {self.setpoint[2]}deg | thrust = {self.setpoint[3]}  |  time {self.timestamp}"


@dataclass
class PauseSetpoint(CFSetpoint):
    def __init__(self, holdTime: float, priority: int = 0) -> None:
        """Add a pause to the command loop

        Args:
            time (float): time in milliseconds to pause for
            priority (int): priority in queue (highest goes first)
        """
        super().__init__()
        # PriorityQueue prioritises lowest number, highest is more logical
        self.priority = -priority
        self.holdTime = holdTime
        self.setpointType = "pause"

    def __repr__(self) -> str:
        return f"Time: {self.holdTime} ms"


class SimpleCommandManager:
    def __init__(self) -> None:
        """Simple queue for different types of commands"""
        self.commands = queue.PriorityQueue(0)

    def addCommand(self, command: CFSetpoint):
        """Add a setpoint to commandqueue"""
        print("Adding: ", command)
        time.sleep(0.0001)
        command.timestamp = time.time()
        self.commands.put(command)
        
    def clearAllCommands(self):
        print("Clearing CommandManager Queue")
        while not self.commands.empty():
            try:
                self.commands.get(False)
            except Exception as e:
                print(f"clearAllCommands error = {e}")
                # continue
            # self.commands.task_done()
        


# NOTE This is the old way that does trajectories by hand, not tested
class TrajectoryManager:
    def __init__(self) -> None:
        self._setpoints = queue.PriorityQueue(0)
        self.setpointsHistory = []  # remove later
        self._waypoints = queue.Queue(0)
        self.waypointsHistory = []  # remove later
        # self._waypoints.put(PositionSetpoint(0,0,0,0))

        self._lastWaypoint = PositionSetpoint(0, 0, 0, 0, 0)

        # fig = plt.figure()
        self.ax = plt.axes(projection="3d")

    def addWaypoint(self, setpoint: PositionSetpoint):
        self._waypoints.put(setpoint)

    def computeTrajectory(self):
        timeMult = 3
        n_order = 5
        n_obj = 3
        sample_rate = 5

        # Add the last waypoint we are flying to to the trajectory as a starting point
        wpts = [self.lastWaypoint.setpoint]
        # initial direction
        v_i = self.lastWaypoint.v
        a_i = self.lastWaypoint.a
        # final direction
        v_e = [0, 0, 0, 0]
        a_e = [0, 0, 0, 0]

        # add wpts to generate trajectory
        while not self._waypoints.empty():
            wpt = self._waypoints.get()
            print(wpt)
            self.waypointsHistory.append(wpt)
            self.lastWaypoint = wpt
            wpts.append([*wpt.setpoint])

        time_set = np.array([x * timeMult for x in range(len(wpts))])

        Matrix_x, Matrix_y, Matrix_z = minimum_snap_traj_p2p(
            wpts, time_set, n_order, n_obj, v_i, a_i, v_e, a_e
        )
        p, v, a, t_list = get_traj(Matrix_x, Matrix_y, Matrix_z, time_set, sample_rate)

        self.lastWaypoint.v = np.array(v).T[-5]

        p = np.array(p).T
        for x, y, z in p:
            self._setpoints.put(PositionSetpoint(x, y, z, 0))
            self.setpointsHistory.append(PositionSetpoint(x, y, z, 0))  # remove later

    def plotTrajectory(self, color="grey"):
        traj = np.array([[*x.setpoint] for x in self.setpointsHistory]).T
        self.ax.scatter(
            traj[0], traj[1], traj[2], marker="x", color=color, s=2, label=color
        )
        # start
        self.ax.scatter(
            traj[0, 0], traj[1, 0], traj[2, 0], marker="o", color="lime", label="start"
        )
        # end
        self.ax.scatter(
            traj[0, -2], traj[1, -2], traj[2, -2], marker="^", color="red", label="end"
        )
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.set_zlim(0, 1)
        # for wpt in wpts:
        #     ax.plot(wpt[0], wpt[1], wpt[2], marker="^")
        # plt.show()

    def showplot(self):
        plt.show()

    @property
    def lastWaypoint(self):
        return self._lastWaypoint

    @lastWaypoint.setter
    def lastWaypoint(self, wpt):
        self._lastWaypoint = wpt


if __name__ == "__main__":
    trajManager = TrajectoryManager()
    for i in range(5):
        values = list(np.random.rand(3)) + [0]
        # values = [i/10,i/10, i/10,0]
        trajManager.addWaypoint(PositionSetpoint(*values))

    trajManager.computeTrajectory()
    trajManager.plotTrajectory()

    for i in range(5):
        values = list(np.random.rand(3)) + [0]
        # values = [1+i/10,1+i/10,1+i/10,0]
        trajManager.addWaypoint(PositionSetpoint(*values))

    trajManager.computeTrajectory()
    trajManager.plotTrajectory("brown")
    # for i in range(5):
    #     values = list(np.random.rand(3)) + [0]
    #     # values = [1+i/10,1+i/10,1+i/10,0]
    #     trajManager.addWaypoint(PositionSetpoint(*values))

    # trajManager.computeTrajectory()
    # trajManager.plotTrajectory("black")
    trajManager.showplot()
