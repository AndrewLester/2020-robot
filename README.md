# 2020 Robot Code
**Robot Code** | [Dashboard](https://github.com/frc1418/2020-dashboard) | [Vision](https://github.com/frc1418/2020-vision)

[![Build Status](https://api.travis-ci.com/frc1418/2020-robot.svg?token=ZK21meVwhYyzjksNsTST&branch=master)](https://travis-ci.com/frc1418/2020-robot)

> Code for Team 1418's 2020 competition robot, which is so far unnamed.

## Deploying onto the robot
Before deploying, you must [install robotpy](http://robotpy.readthedocs.io/en/stable/install/robot.html#install-robotpy) on your robot.

You may then deploy code at any time:

	python3 src/robot.py deploy

We recommend using our tool, `dep`, [here](https://github.com/frc1418/dep). It simplifies your deploy process and removes pesky steps like manually changing your WiFi network.

## Testing/Simulation
You may run the `pyfrc` simulator to test this code thus:

    python3 src/robot.py sim

## Controls
We use three total joysticks to control the robot:

* 2 x **Logitech Attack 3** (`joystick_left` and `joystick_right`)
* 1 x **Logitech Extreme 3D Pro** (`joystick_alt`)

<img src="res/ATK3.png" height="600"><img src="res/X3D.png" height="600">

# Basic Setup

## Setting up `git` hooks:

`git` hooks change the process of committing by adding processes before or after the process of committing. After cloning, you should run

	./setup.sh

This will set up hooks to run tests before committing to help avoid easy-to-fix errors in the code.

## Installing Dependencies

The `requirements.txt` file holds all of the programs and their versions needed to run this code. After activating your virtual environment with run

	pip3 install -r requirements.txt

In the `tests` folder there is another requirements file that needs to be run. To do so, run

	pip3 install -r tests/requirements.txt

## Setting up entry points

Entry points allow us to add sub-commands to the overarching RobotPY framework. To load them, move to the root directory, activate your virtual environment and run

	pip install -e .

This will set up the entry points that we put in place to facilitate off-robot functions of our code. So far, we use entry points for:
* Generating motion profile trajectories (generate)

These entry points can be run like so

    python3 src/robot.py <entry_point_name>

## Using a virtual environment

A virtual environment allows you to store dependencies in a project folder, meaning that they do not affect your computer's python installation. To create a virtual environment, first install the virtualenv package

	pip3 install virtualenv --user

Now, to create a virtual environment in a project folder, navigate into the folder and run

	python3 -m virtualenv venv

After this command, your virtual environment  will have been created. It now resides in the "venv" folder inside of your project. However, it is not "enabled" by default, meaning any packages you install right now will still go to your global python install. To activate the environment, run

  	. venv/bin/activate

You will be able to tell that it worked by looking at the additional prefix that should appear before your main bash prefix. It should look something like

	(venv)

After you're done working on a specific project, and want to use your global python installation or activate a different virtual environment, you can type

	deactivate

This command will deactivate your current virtual environment from any location in the terminal.

## File Structure

    src/
    	The robot code lives here.
        automations/
            Automatic scripts for performing common functions.
        autonomous/
            Autonomous modes.
        common/
            New robotpy components.
        components/
            Abstractions for major robot systems.
        controllers/
            Software implementations not corresponding to physical robot components.
	tests/
		py.test-based unit tests that test the code and can be run via pyfrc.

## Authors
* Team 1418

## Licensing
In-season, use of this software is restricted by the FRC rules. After the season ends, the [MIT License](LICENSE) applies instead.
