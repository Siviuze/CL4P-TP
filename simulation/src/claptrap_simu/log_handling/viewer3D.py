#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
    Display simulation results in Meshcat. One or several files can be passed as argument and will be plotted in
    the same window.
'''

import argparse
import meshcat
import numpy as np
import pinocchio as pnc
import matplotlib.colors as colors
import pkg_resources
import os

from .log_loader import LogLoader


def display_3d(files):
    '''
    Display specified logfiles in Meshcat. Color of each file is picked from Matplotlib default color cycle
    @param files List of files to display.
    '''
    # Load default URDF.
    urdf_path = pkg_resources.resource_filename('claptrap_simu', 'data/claptrap.urdf')
        
    display_robots = []
    for f in files:
        # Create robot for this file
        robot = pnc.RobotWrapper.BuildFromURDF(urdf_path, [os.path.dirname(urdf_path)], root_joint=None)
        
        # Load file data
        log = LogLoader(f)
        file_data = np.matrix([log.data["Claptrap.q" + str(i)] for i in range(robot.model.nq)]).T
        display_robots.append({"robot": robot, "data" : file_data, "dt" : log.data["time"][1] - log.data["time"][0]})
    
    # Find smallest dt
    dt_smallest = min([r["dt"] for r in display_robots])
    
    # Create viewer
    viewer = meshcat.visualizer.Visualizer("tcp://127.0.0.1:6000")
    anim = meshcat.animation.Animation(default_framerate = 1.0 / dt_smallest)
    
    with anim.at_frame(viewer, 0) as frame:
        for i in range(len(display_robots)):
            display_robots[i]["robot"].initMeshcatDisplay(viewer, str(i), colors.to_rgba("C" + str(i % 10), alpha=0.8))
            display_robots[i]["robot"].viz.viewer = frame
            display_robots[i]["robot"].display(display_robots[i]["data"][0].T)
    
    # Play animation until end of last logfile.
    # Keep in memory a running index for each robot, updating it as time moves forward, until the end of every log.
    replay_index = [0] * len(display_robots)
    current_index = 1
    
    done = False
    while not done:
        done = True
        with anim.at_frame(viewer, current_index) as frame:
            for i in range(len(display_robots)):
                # If we are already at the end of this log file, don't do anything.
                if replay_index[i] < len(display_robots[i]["data"]) - 1:
                    # We are not done replaying.
                    done = False
                    # Check to see if we need to increase the index
                    if dt_smallest * current_index > display_robots[i]["dt"] *  replay_index[i]:
                        replay_index[i] += 1
                display_robots[i]["robot"].viz.viewer = frame
                display_robots[i]["robot"].display(display_robots[i]["data"][replay_index[i]].T)
        current_index += 1
    # Now replay animation in viewer.
    viewer.set_animation(anim)
    

def main():
    description_str = "Replay a simulation in Meshcat. Give as input a list to files to display."
    parser = argparse.ArgumentParser(description = description_str, formatter_class = argparse.RawTextHelpFormatter)
    # No args for now...
    args, files = parser.parse_known_args()
    display_3d(files)
    
if __name__ == "__main__":
    main()
