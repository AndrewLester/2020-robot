import os
import sys
import pickle
import pathfinder as pf
from . import robotpy_entry_point
import math


WHEELBASE_WIDTH = 0.557784  # Units: meters
TRAJECTORY_DIRECTORY = 'trajectories'
PICKLE_FILE = os.path.join(os.path.dirname(sys.modules['__main__'].__file__), TRAJECTORY_DIRECTORY, 'trajectories.pickle')
MAX_GENERATION_VELOCITY = 1.03632  # Units: m/s
MAX_GENERATION_ACCELERATION = 2.5908  # Units: m/s^2
MAX_GENERATION_JERK = 4.57200  # Units: m/s^3

trajectories = {
    "charge": [
        pf.Waypoint(0, 0, 0),
        pf.Waypoint(3, 0, 0)
    ],
    "diagonal_higher": [
        pf.Waypoint(0, 0, 0),  # Waypoints are relative to first, so start at 0, 0, 0
        pf.Waypoint(15, 8, 0)
    ],
    "cargo_ship": [
        pf.Waypoint(0, 0, 0),
        pf.Waypoint(7.33, 0, 0)
    ],
    "left-side": [
        pf.Waypoint(0, 0, 0),
        pf.Waypoint(6, 6, math.pi / 4)
    ],
    "turn": [
        pf.Waypoint(0, 0, 0),  # Waypoints are relative to first, so start at 0, 0, 0
        pf.Waypoint(0.5, 0, math.pi / 4)
    ]
}


@robotpy_entry_point(name='TrajectoryGenerator')
def generate_trajectories(options, robot_class):
    """
    Generate trajectory from waypoints.
    :param options: Options for the entry point being called
    :param robot_class: The class of the robot being run
    """
    print('Generating Trajectories...')
    generated_trajectories = {}

    for trajectory_name, trajectory in trajectories.items():
        print(f'Trajectory: {trajectory_name} ...', end='')
        generated_trajectory = pf.generate(
            trajectory,
            pf.FIT_HERMITE_CUBIC,
            pf.SAMPLES_HIGH,
            dt=0.02,  # 20ms
            max_velocity=MAX_GENERATION_VELOCITY,  # These are in ft/sec and
            max_acceleration=MAX_GENERATION_ACCELERATION,  # set the units for distance to ft.
            max_jerk=MAX_GENERATION_JERK
        )[1]  # The 0th element is just info

        modifier = pf.modifiers.TankModifier(generated_trajectory).modify(WHEELBASE_WIDTH)

        generated_trajectories[trajectory_name] = (
            modifier.getLeftTrajectory(),
            modifier.getRightTrajectory()
        )
        print('Done')

    with open(PICKLE_FILE, 'wb') as f:
        pickle.dump(generated_trajectories, f)

    print('Finished.')
