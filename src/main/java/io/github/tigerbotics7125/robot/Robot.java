/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import io.github.tigerbotics7125.tigerlib.CommandRobot;

public class Robot extends CommandRobot {

    public static final RobotContainer kRobotContainer = new RobotContainer();

    @Override
    public void robotInit() {
	if (Robot.isSimulation())
	    this.addPeriodic(kRobotContainer::simulationPeriodic, kDefaultPeriod);
    }
}
