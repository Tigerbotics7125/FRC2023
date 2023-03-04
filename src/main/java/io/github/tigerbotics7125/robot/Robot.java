/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.github.tigerbotics7125.tigerlib.CommandRobot;

public class Robot extends CommandRobot {

    public static final RobotContainer mRobotContainer = new RobotContainer();

    private double mMatchStartTime = 0.0;
    private String mMatchTime = "N/A";

    @Override
    public void robotInit() {
        OperatorInterface.init();
        CANSparkMax.enableExternalUSBControl(true);
    }

    @Override
    public void robotPeriodic() {
        mRobotContainer.periodic();
        OperatorInterface.update();
        Shuffleboard.update();
        SmartDashboard.updateValues();
    }

    @Override
    public void autonomousInit() {
        OperatorInterface.startMatchSegment();
        CommandScheduler.getInstance().schedule(mRobotContainer.getAutoCommand());
    }

    @Override
    public void teleopInit() {
        OperatorInterface.startMatchSegment();
    }

    @Override
    public void simulationInit() {
        // Disable joystick warnings in sim.
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void simulationPeriodic() {
        mRobotContainer.simulationPeriodic();
        REVPhysicsSim.getInstance().run();
    }
}
