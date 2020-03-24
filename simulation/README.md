# Simulation

This folder contains a python simulation for Claptrap control, relying on the open-source
kinematics and dynamics library ```pinocchio```.

This simulation basically takes a URDF representation of Claptrap as input ; forward dynamics is done
using minimal coordinates, assuming a slipless contact of the wheel with the ground.
For more details about the simulation and the applied controllers, see ControlDocumentation.pdf

## Installation

As usual, install the requirements then the package itself (preferably in a virtual env) using:

```
pip install -r requirements.txt
python setup.py install
```

This code also depends ```pinocchio``` and its python bindigns, see 
[github](https://github.com/stack-of-tasks/pinocchio) for installation instructions.

## Running the simulation

Example scripts and true simulation scenarios are present in the scenarios folder. For instance, open a meshcat server
using:

```
meshcat-server
```

Then in another terminal run:

```
python scenarios/example_sagittal.py
```

This scripts simulates a pure sagittal motion, using a simple PD controller. The results are saved in a log file,
and then replayed in the meshcat viewer using ```claptrap_replay```. Note that this tool can be called from the command
line:

```
claptrap_replay /path/to/log.csv
```

For analysis, see ```claptrap_plotter``` for a generic plotter of a CSV file.
