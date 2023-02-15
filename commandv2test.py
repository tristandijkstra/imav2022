from cftoolbox import drone
from cftoolbox.logprofiles import gimbalProfileSet, posProfileSet

if __name__ == "__main__":
    d = drone.Drone(uri="radio://0/80/2M/E7E7E7E842",logProfiles=posProfileSet, plottedProfiles=["z", "x", "y", "yaw"])


    # d.addcommands(circle)

    input("Press enter to start\n")
    d.start_super_commander()


    input("Press enter to land\n")
    d.disconnect()

# 1-115, 1.5=85
# looking left is positive
# 0xE7E7E7E842
