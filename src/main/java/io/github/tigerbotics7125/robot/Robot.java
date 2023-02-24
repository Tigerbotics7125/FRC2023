/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        // mRobotContainer.getAutonomousCommand().schedule();
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
