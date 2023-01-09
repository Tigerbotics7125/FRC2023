/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import io.github.tigerbotics7125.tigerlib.CommandRobot;

public class Robot extends CommandRobot {

    public static final RobotContainer mRobotContainer = new RobotContainer();

    // this value should be true out on the field, where we are using replay.
    // but false when using the sim for normal uses lol.
    private final boolean useReplay = false;

    @Override
    public void robotInit() {

    }

    @Override
    public void simulationPeriodic() {
	mRobotContainer.simulationPeriodic();
    }
}
