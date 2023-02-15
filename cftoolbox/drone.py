import queue
import threading
import multiprocessing
import time
# import numpy as np
import math
import pandas as pd
import cv2
import cflib.crtp  # Scans crazyflies available for communication
from cflib.crazyflie import Crazyflie  # Object used to send data to a crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils.power_switch import PowerSwitch
from cftoolbox.trajectory import PositionSetpoint, PauseSetpoint, AnglesSetpoint
# from cftoolbox import commandRecipes as recipes
import cftoolbox.colourTools as ct

from . import gap8Camera
from .gap8Camera import Camera
from .logprofiles import defaultProfileSet, posProfileSet
from .plotting import ProcessPlotter
from .trajectory import RelativePositionSetpoint, SimpleCommandManager, CFSetpoint


TIMETOTRAJECTORIES = 10
STOPMINUTES = 1

class Drone:
    def __init__(
        self,
        uri="radio://0/80/2M/E7E7E7E7E7",
        logProfiles=defaultProfileSet,
        plottedProfiles=[],
        gimbalMode=False,
        logBattery=False,
        enablePlotting=True,
        superCommander=True
    ) -> None:
        # Init Parameters
        self.STARTTIME = None
        self.GATEA = None
        self.GATEB = None
        self.GATELASTFLOWNTO = None
        self.uri = uri
        self._logProfiles = logProfiles
        self.plottedProfiles = plottedProfiles
        self.gimbalMode = gimbalMode
        self.logBattery = logBattery
        self.enablePlotting = enablePlotting

        # Set up submodules
        cflib.crtp.init_drivers()
        self._cf = Crazyflie(rw_cache="./cache")
        self._commandLoop = threading.Thread(target=self._command_loop_new)
        self.superCommander = threading.Thread(target=self.super_commander)
        self.hl_commander = self._cf.high_level_commander
        # self._cf.param.set_value('commander.enHighLevel', '1')
        self.trajectorymanager = SimpleCommandManager()

        self._commandLoopOn = True
        self.SUPERCOMMANDERSTATUS = "circle"
        self._supercommanderOn = True
        self.current_setpoint = None

        # Add callbacks
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        # setup position dictionary
        self.pos = {"x" : None, "y" : None, "z" : None}

        # plotting logs
        self.finishedPlotting = False
        if self.enablePlotting:
            self.refreshTime = 50
            self.plot_queue = multiprocessing.Queue()
            self.plotter = ProcessPlotter(
                self.refreshTime, self.plottedProfiles, logProfiles
            )
            self.plot_process = multiprocessing.Process(
                target=self.plotter, args=(self.plot_queue,)
            )

        # Initiate Connection
        self.connect()

        # Setup camera
        self.cameraData = {}
        self.camera_queue = multiprocessing.Queue(0)

        args = gap8Camera.getArgs()
        client = gap8Camera.connectSocket(args)
        self.camera_process = multiprocessing.Process(target=Camera,args=(client,self.camera_queue))
        self.camera_process.start()

    # Basic Callbacks
    def _connected(self, uri: str):
        print(f"Connected: {uri}")
        # self._cf.param.add_update_callback(group='flightmode', name='poshold',
        #                                    cb=self._param_kp_callback)
        if self.enablePlotting:
            self._startLoggerProfiles()
            self.plot_process.start()

    # def startttt(self):
    #     # self._cf.param.set_value('stabilizer.controller', '2')
    #     # print("flow deck:", self._cf.param.get_value("deck.bcFlow2", 10))
    #     self._cf.param.set_value("flightmode.poshold", "True")

    def _connection_failed(self, uri: str, msg: str):
        print(f"Connection failed: {uri} | {msg}")

    def _connection_lost(self, uri: str, msg: str):
        print(f"Connection lost: {uri} | {msg}")
        self._commandLoopOn = False
        self._commandLoop.join()

    def _disconnected(self, uri: str):
        print(f"Disconnected: {uri}")

    # Connection:
    def connect(self):
        self._cf.open_link(self.uri)

    def disconnect(self):
        print("Disconnecting...")
        if not self.gimbalMode:
            print("Landing")
            self.hl_commander.land(0.05, 2)
            time.sleep(3)
            # self._cf.commander.send_setpoint(0, 0, 0, 0)
        else:
            self._cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.2)
        self._commandLoopOn = False
        self._commandLoop.join()
        self.finishedPlotting = True
        self.plot_process.kill()
        PowerSwitch(self.uri).stm_power_cycle()
        self._cf.close_link()

    def _map_thrust(self, thrust):
        """Remap thrust from 0 to 50000"""
        return int(0 + (thrust) * (49999 - 0) / (100))


    def _command_loop_new(self):
        print("Command Loop V2 Started")
        if not self.gimbalMode:
            self.hl_commander.takeoff(1, 7)
            time.sleep(7.1)
            print("Takeoff complete")
        while self._commandLoopOn:
            try:
                command = self.trajectorymanager.commands.get(timeout=0.1)
                if command.setpointType == "angles":
                    self._cf.commander.send_setpoint(0, 0, 0, 0)
                    start = time.time()
                    while time.time() - start < (command.holdTime / 1000):
                        self._cf.commander.send_setpoint(*command.setpoint)

                elif command.setpointType == "pause":
                    print(f"Sleeping for {command.holdTime} | {time.time()}")
                    time.sleep(command.holdTime)

                elif command.setpointType == "relative":
                    self.hl_commander.go_to(*command.setpoint, command.timeTo, relative=True)
                    time.sleep(command.timeTo)

                elif command.setpointType == "position":
                    self.hl_commander.go_to(*command.setpoint, command.timeTo)
                    time.sleep(command.timeTo)
                else:
                    print("Error in commandloop, command not recognised")
            except queue.Empty as e:
                pass
            except Exception as e:
                print(f"An Error occured in the command loop: {e}")
                if not self.gimbalMode:
                    self.hl_commander.land(0.05, 4)
                self._commandLoopOn = False
                self.disconnect()
                self._commandLoop.terminate()
            time.sleep(0.001)
        print("Command Loop Finished")

    def super_commander(self):

        #Start the low level commander before you start the supercommander
        self.startCommander()
        time.sleep(7.5)
        self.STARTTIME = time.time()

        while self._supercommanderOn:
            self.cameraData = self.camera_queue.get()
            try:
                if self.STARTTIME - time.time() > STOPMINUTES*60:
                    print("TIME IS UP")
                    self._supercommanderOn = False
                    self.disconnect()

                if  self.cameraData["camStatus"] == True:
                    print("AVOIDING")
                    self.SUPERCOMMANDERSTATUS = "circle"
                    quarterturn = [RelativePositionSetpoint(0,0,0, yaw, 1) for yaw in range(0, 91, 10)]
                    avoid = quarterturn + [RelativePositionSetpoint(0.2,0,0, 0, 1, currentYAW=self.pos["yaw"])]
                    self.addcommands(avoid)
                    # time.sleep(36)
                    continue


                if self.SUPERCOMMANDERSTATUS == "circle":
                    print("CIRCLING")
                    self.SUPERCOMMANDERSTATUS = "scanning"
                    halfturn = [RelativePositionSetpoint(0,0,0, yaw, 1) for yaw in range(0, 121, 10)]
                    
                    circle = halfturn + [RelativePositionSetpoint(0.5,0,0, 0, 2, currentYAW=self.pos["yaw"])]
                    # time.sleep(2)

                    self.addcommands(circle)
                    # time.sleep(36)
                    continue
                
                if self.SUPERCOMMANDERSTATUS == "scanning":
                    print("SCANNING")
                    if self.cameraData["targetOnScreen"] == True:
                        self.trajectorymanager.clearAllCommands()
                        commands = [RelativePositionSetpoint(0.4,0,0, 0, 3, currentYAW=self.pos["yaw"])]
                        time.sleep(3)
                        self.addcommands(commands)
                        self.SUPERCOMMANDERSTATUS = "flyThroughGate"

                    continue
                    # else
                    

                elif self.SUPERCOMMANDERSTATUS == "flyThroughGate":
                    self.trajectorymanager.clearAllCommands()
                    print("flyThroughGate")
                    camx, camy = self.cameraData["center"]

                    if (self.cameraData["distance"] is not None) and abs(camx) > 0.07:
                        mult = 4
                        print(f"camx = {camx} | camy = {camy} | distance = {self.cameraData['distance']}")
                        command = [RelativePositionSetpoint(0,0,0, -mult*camx,timeTo=1, currentYAW=self.pos["yaw"])]
                        self.addcommands(command)
                        continue
                    elif (self.cameraData["distance"] is None):
                        print("Marker Lost")
                        command = [RelativePositionSetpoint(0.4,0,0,0,timeTo=1, currentYAW=self.pos["yaw"])]
                        time.sleep(1)
                        self.addcommands(command)
                        self.SUPERCOMMANDERSTATUS = "circle"
                        continue
                    else:
                        print("GETTING CLOSE")
                        if self.GATEA is None:
                            # self.GATEA = tuple(self.pos.values())
                            # self.GATELASTFLOWNTO = "A"
                            distMove = self.cameraData["distance"] if self.cameraData["distance"] is not None else 0.8
                            command = [RelativePositionSetpoint(distMove+0.5,0,0,0,TIMETOTRAJECTORIES, currentYAW=self.pos["yaw"]),
                                    RelativePositionSetpoint(0,0.5,0, 0,timeTo=6, currentYAW=self.pos["yaw"])]
                            # print("FINDING B")
                            self.addcommands(command)
                            time.sleep(TIMETOTRAJECTORIES+6)
                            self.trajectory.clearAllcommands()
                            self.SUPERCOMMANDERSTATUS = "circle"
                            continue

                        elif self.GATEB is None:
                            self.GATEB = tuple(self.pos.values())
                            self.GATELASTFLOWNTO = "B"
                            distMove = self.cameraData["distance"] if self.cameraData["distance"] is not None else 0.8
                            command = [RelativePositionSetpoint(distMove+0.5,0,0, 0,TIMETOTRAJECTORIES, currentYAW=self.pos["yaw"]),
                                       PositionSetpoint(self.GATEA[0], self.GATEA[1], self.GATEA[2], 0 , 20)]
                            print("GOING BACK TO A")
                            self.addcommands(command)
                            time.sleep(2*TIMETOTRAJECTORIES)
                            self.trajectory.clearAllcommands()
                            self.SUPERCOMMANDERSTATUS = "circle"
                            continue

                                       
                        else:
                            nextGate = self.GATEA if (self.GATELASTFLOWNTO == "B") else self.GATEB
                            print(f"NEXT GATE = {nextGate}")
                            print(f"LAST GATE = {self.GATELASTFLOWNTO}")
                            distMove = self.cameraData["distance"] if self.cameraData["distance"] is not None else 0.8
                            command = [RelativePositionSetpoint(distMove+0.5,0,0, 0,TIMETOTRAJECTORIES, currentYAW=self.pos["yaw"]),
                                       PositionSetpoint(nextGate[0], nextGate[1], nextGate[2], 0 , 20)]
                            self.addcommands(command)
                            time.sleep(2*TIMETOTRAJECTORIES)
                            self.trajectory.clearAllcommands()
                            self.SUPERCOMMANDERSTATUS = "circle"
                            continue


                    continue


            except Exception as e:
                print(f"Error in SuperCommander {e}")

    def addcommands(self, commands: list):
        """Add list of CFsetpoint objects to command queue

        Args:
            commands (list): list of CFSetpoint objects
        """
        for com in commands:
            self.trajectorymanager.addCommand(com)

    def start_super_commander(self):
        self.superCommander.start()

    def startCommander(self):
        self._commandLoop.start()

    def _startLoggerProfiles(self):
        # Profile loggers
        for profname, profile in self._logProfiles.items():
            try:
                p = LogConfig(name=profname, period_in_ms=self.refreshTime)
                for name, ctype in profile.items():
                    p.add_variable(name, ctype)
                self._startLogger(p)
            except Exception as e:
                print(f"Exception occured in _startLoggerProfiles: {e}")

        # Battery status
        if self.logBattery:
            bat = LogConfig(name="battery", period_in_ms=2000)
            bat.add_variable("pm.vbat", "FP16")
            self._cf.log.add_config(bat)
            bat.data_received_cb.add_callback(self._log_bat)
            bat.error_cb.add_callback(self._log_error)
            bat.start()

    def _log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print("Error when logging %s: %s" % (logconf.name, msg))

    def _log_data(self, timestamp, data, logconf):
        """Log callback, send log data to plotter."""
        if self.finishedPlotting:
            self.plot_queue.put(None)
        else:
            if "stateEstimate.x" in data:
                self.pos["x"] = data["stateEstimate.x"]
            if "stateEstimate.y" in data:
                self.pos["y"] = data["stateEstimate.y"]
            if "stateEstimate.z" in data:
                self.pos["z"] = data["stateEstimate.z"]
            if "stabilizer.yaw" in data:
                self.pos["yaw"] = data["stabilizer.yaw"]
            data = timestamp, data
            # self.plot_queue.put(data)

    def _log_bat(self, timestamp, data, logconf):
        """Log callback, send log data to plotter."""
        bat = round((data["pm.vbat"]-2.4) / (4.2-2.4) * 100, 2)
        print(f"[{timestamp}] - Battery Level = {bat}% ({data['pm.vbat']})")

    def _startLogger(self, logProfile):
        try:
            self._cf.log.add_config(logProfile)
            logProfile.data_received_cb.add_callback(self._log_data)
            logProfile.error_cb.add_callback(self._log_error)
            logProfile.start()
        except KeyError as e:
            print(
                "Could not start log configuration,"
                "{} not found in TOC".format(str(e))
            )
        except AttributeError as e:
            print(f"Bad logging configuration: {e}")
