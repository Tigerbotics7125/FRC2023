/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import static io.github.tigerbotics7125.robot.Constants.OperatorInterface.kDriverPort;
import static io.github.tigerbotics7125.robot.Constants.OperatorInterface.kOperatorPort;
import static io.github.tigerbotics7125.tigerlib.input.trigger.Trigger.ActivationCondition.*;

import io.github.tigerbotics7125.robot.subsystem.Drivetrain;
import io.github.tigerbotics7125.tigerlib.input.controller.XboxController;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger;
import io.github.tigerbotics7125.tigerlib.util.JoystickUtil;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
    XboxController mDriver = new XboxController(kDriverPort);
    XboxController mOperator = new XboxController(kOperatorPort);
    Trigger mRioUserButton = new Trigger(RobotController::getUserButton);

    Drivetrain mDrivetrain = new Drivetrain();
    // Vision mVision = new Vision();

    Field2d mField = new Field2d();

    public RobotContainer() {
        initDriver();
        initOperator();
        initTriggers();
        initDefaults();
        // something

    }

    private void initDriver() {
        mDriver.rb()
                .activate(ON_RISING)
                .trigger(
                        new InstantCommand(
                                () -> {
                                    mDrivetrain.toggleTurnMode();
                                    System.out.println("turningTrigger");
                                }));
        mDriver.start().activate(ON_RISING).trigger(mDrivetrain::resetGyro);
    }

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
                            Pair<Double, Double> leftJoystick =
                                    JoystickUtil.mapToCircle(
                                            mDriver.leftX().get(), mDriver.leftY().get());
                            Pair<Double, Double> rightJoystick =
                                    JoystickUtil.mapToCircle(
                                            mDriver.rightX().get(), mDriver.rightY().get());
                            mDrivetrain.drive(
                                    leftJoystick.getSecond(),
                                    leftJoystick.getFirst(),
                                    rightJoystick.getFirst(),
                                    rightJoystick.getSecond());
                        },
                        mDrivetrain));
    }

    public void periodic() {
        // mVision.getTargets().forEach(mDrivetrain::feedVisionTarget);
    }

    public void simulationPeriodic() {
        Pose2d robotPose = mDrivetrain.getPose();
        mField.setRobotPose(robotPose);
        mField.getObject("LeftJoystick")
                .setPose(
                        new Pose2d(
                                robotPose.getX() + mDriver.leftY().get(),
                                robotPose.getY() + mDriver.leftX().get(),
                                new Rotation2d()));
        mField.getObject("RightJoystick")
                .setPose(
                        new Pose2d(
                                robotPose.getX() + mDriver.rightY().get(),
                                robotPose.getY() + mDriver.rightX().get(),
                                new Rotation2d()));
        Pair<Double, Double> leftJoystick =
                JoystickUtil.mapToCircle(mDriver.leftX().get(), mDriver.leftY().get());
        Pair<Double, Double> rightJoystick =
                JoystickUtil.mapToCircle(mDriver.rightX().get(), mDriver.rightY().get());
        mField.getObject("LeftJoystickMapped")
                .setPose(
                        new Pose2d(
                                robotPose.getX() + leftJoystick.getSecond(),
                                robotPose.getY() + leftJoystick.getFirst(),
                                new Rotation2d()));
        mField.getObject("RightJoystickMapped")
                .setPose(
                        new Pose2d(
                                robotPose.getX() + rightJoystick.getSecond(),
                                robotPose.getY() + rightJoystick.getFirst(),
                                new Rotation2d()));
        SmartDashboard.putData(mField);
    }
}
