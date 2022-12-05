/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import static io.github.tigerbotics7125.robot.Constants.OperatorInterface.kDriverPort;
import static io.github.tigerbotics7125.robot.Constants.OperatorInterface.kOperatorPort;

import io.github.tigerbotics7125.robot.subsystem.Drivetrain;
import io.github.tigerbotics7125.robot.subsystem.Vision;
import io.github.tigerbotics7125.tigerlib.input.controller.XboxController;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
    XboxController mDriver = new XboxController(kDriverPort);
    XboxController mOperator = new XboxController(kOperatorPort);
    Trigger mRioUserButton = new Trigger(RobotController::getUserButton);

    Drivetrain mDrivetrain = new Drivetrain();
    Vision mVision = new Vision();

    Field2d mField = new Field2d();

    public RobotContainer() {
        initDriver();
        initOperator();
        initTriggers();
        initDefaults();
        // something

    }

    private void initDriver() {}

    private void initOperator() {}

    private void initTriggers() {
        mRioUserButton.trigger(mDrivetrain::resetGyro);
        new Trigger(RobotState::isDisabled).trigger(mDrivetrain::setCoastMode);
        new Trigger(RobotState::isEnabled).trigger(mDrivetrain::setBrakeMode);
    }

    private void initDefaults() {
        mDrivetrain.setDefaultCommand(
                new RunCommand(
                        () -> {
                            // mDriver.rightX().setCleaner((x) -> JoystickUtil.deadband(x, .5));
                            // mDriver.rightY().setCleaner((x) -> JoystickUtil.deadband(x, .5));
                            mDrivetrain.driveFaceAngle(
                                    mDriver.leftY().get(),
                                    mDriver.leftX().get() * -1,
                                    mDriver.rightX().get(),
                                    mDriver.rightY().get());
                        },
                        mDrivetrain));
    }

    public void periodic() {
        mVision.getTargets().forEach(mDrivetrain::feedVisionTarget);
    }

    public void simulationPeriodic() {
        mField.setRobotPose(mDrivetrain.getPose());
        SmartDashboard.putData(mField);
    }
}
