/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.tigerbotics7125.tigerlib.CommandRobot;

public class Robot extends CommandRobot {

    public static final RobotContainer mRobotContainer = new RobotContainer();

    @Override
    public void robotInit() {}

    @Override
    public void robotPeriodic() {
        mRobotContainer.periodic();
        Shuffleboard.update();
        SmartDashboard.updateValues();
    }

    @Override
    public void autonomousInit() {
       // mRobotContainer.getAutonomousCommand().schedule();
    }

    @Override
    public void simulationPeriodic() {
        mRobotContainer.simulationPeriodic();
        REVPhysicsSim.getInstance().run();
    }
}
