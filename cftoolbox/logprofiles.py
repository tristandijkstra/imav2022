stabilizerProfile = {
    # stab position
    "stabilizer.roll": "float",
    "stabilizer.pitch": "float",
    "stabilizer.yaw": "float",
    "stabilizer.thrust": "float",
}
controllerProfile = {
    # cmd position
    "controller.roll": "float",
    "controller.pitch": "float",
    "controller.yaw": "float",
    "controller.cmd_thrust": "float",
}
motorProfile = {
    # individual motor pwm
    "motor.m1": "uint32_t",
    "motor.m2": "uint32_t",
    "motor.m3": "uint32_t",
    "motor.m4": "uint32_t",
}

positionProfile = {
    # individual motor pwm
    "stateEstimate.x": "float",
    "stateEstimate.y": "float",
    "stateEstimate.z": "float",

}

miscProfile = {
    # misc
    "pm.vbat": "FP16",
}


defaultProfileSet = {
    "stabilizer": stabilizerProfile,
    "controller": controllerProfile,
    "motor": motorProfile,
    "misc": miscProfile,
    "position": positionProfile,
}



# PID FOR GIMBAL PROFILE

rollProfile = {
    "controller.roll": "float",
    "stabilizer.roll": "float",
}
pitchProfile = {
    "controller.pitch": "float",
    "stabilizer.pitch": "float",
}

yawProfile = {
    "controller.yaw": "float",
    "stabilizer.yaw": "float",
}
thrustProfile = {
    "controller.cmd_thrust": "float",
    "stabilizer.thrust": "float",
}

gimbalProfileSet = {
    "roll": rollProfile,
    "pitch": pitchProfile,
    "yaw": yawProfile,
    "thrust": thrustProfile,
}


# PID FOR GIMBAL PROFILE

xposProfile = {
    "stateEstimate.x": "float",
    "ctrltarget.x": "float",
}

yposProfile = {
    "stateEstimate.y": "float",
    "ctrltarget.y": "float",
}

zposProfile = {
    "stateEstimate.z": "float",
    "ctrltarget.z": "float",
}
yawposProfile = {
    "stabilizer.yaw": "float",
    # "ctrltarget.yaw": "float",
}


posProfileSet = {
    "z": zposProfile,
    "x": xposProfile,
    "y": yposProfile,
    "yaw": yawposProfile,
}