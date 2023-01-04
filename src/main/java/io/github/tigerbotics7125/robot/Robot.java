/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import io.github.tigerbotics7125.tigerlib.TigerLib;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

    public static final RobotContainer kRobotContainer = new RobotContainer();

    @Override
    public void robotInit() {
	TigerLib.init();
	this.addPeriodic(kRobotContainer::periodic, kDefaultPeriod);
	this.addPeriodic(TigerLib::periodic, kDefaultPeriod);
	if (Robot.isSimulation())
	    this.addPeriodic(kRobotContainer::simulationPeriodic, kDefaultPeriod);
    }

    @Override
    public void robotPeriodic() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}
}
