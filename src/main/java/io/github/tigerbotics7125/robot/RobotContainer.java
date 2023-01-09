/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import static io.github.tigerbotics7125.robot.Constants.OperatorInterface.kDriverPort;
import static io.github.tigerbotics7125.robot.Constants.OperatorInterface.kOperatorPort;
import static io.github.tigerbotics7125.robot.Constants.Vision.kTagLayout;

import io.github.tigerbotics7125.robot.subsystem.Drivetrain;
import io.github.tigerbotics7125.robot.subsystem.Drivetrain.TurningMode;
import io.github.tigerbotics7125.robot.subsystem.Vision;
import io.github.tigerbotics7125.tigerlib.input.controller.XboxController;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger.ActivationCondition;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.ArrayList;
import java.util.List;

public class RobotContainer {

    XboxController mDriver = new XboxController(kDriverPort);
    XboxController mOperator = new XboxController(kOperatorPort);
    Trigger mRioUserButton = new Trigger(RobotController::getUserButton);

    Drivetrain mDrivetrain = new Drivetrain();
    Vision mVision = new Vision();

    Field2d mField = new Field2d();
    AprilTagLayout mTagLayout = new AprilTagLayout(kTagLayout);

    boolean correcting = false;

    public RobotContainer() {

	initDriver();
	initOperator();

	initTriggers();

	initDefaultCommands();

	if (Robot.isSimulation()) {
	    List<Pose2d> tagPoses = new ArrayList<>();
	    mField.getObject("16h5Tags").setPoses(tagPoses);
	    mTagLayout.getTags().forEach((t) -> {
		mVision.addAprilTag(t.getID(), t.getPose());
		tagPoses.add(t.getPose().toPose2d());
	    });
	}
    }

    /** Initialize Driver Triggers. */
    private void initDriver() {
	mDriver.start().trigger(mDrivetrain::resetGyro);
	mDriver.lb().trigger(() -> mDrivetrain.setTurningMode(TurningMode.JOYSTICK_DIRECT));
	mDriver.rb().trigger(() -> mDrivetrain.setTurningMode(TurningMode.JOYSTICK_ANGLE));
	mDriver.y().trigger(() -> mDrivetrain.setTurningMode(TurningMode.HEADING_LOCK));

	mDriver.back().trigger(() -> correcting = true);
	mDriver.back().activate(ActivationCondition.ON_FALLING).trigger(() -> correcting = false);
    }

    /** Initialize Operator Triggers. */
    private void initOperator() {}

    /** Initialize non-OI Triggers. */
    private void initTriggers() {
	new Trigger(RobotController::getUserButton).trigger(() -> mDrivetrain.setPose(new Pose2d(), new Rotation2d()));
	new Trigger(RobotState::isDisabled).trigger(mDrivetrain::setCoastMode);
	new Trigger(RobotState::isEnabled).trigger(mDrivetrain::setBrakeMode);

	new Trigger(mVision::hasTargets).trigger(() -> {
	    double timestamp = mVision.getTimestamp();
	    mVision.getRobotPoseEstimates().forEach((pose) -> {
		mDrivetrain.addVisionMeasurement(pose.toPose2d(), timestamp);
	    });
	});
    }

    /** Set Subsystem's default commands. */
    private void initDefaultCommands() {
	mDrivetrain.setDefaultCommand(Commands.run(() -> mDrivetrain.drive(mDriver.leftY().get(), mDriver.leftX().get(),
		mDriver.rightX().get(), mDriver.rightY().get()), mDrivetrain));
    }

    /** Periodic call, only runs during simulation. */
    public void simulationPeriodic() {
	// update sim vision with our robot pose.
	Pose2d robotPose = mDrivetrain.getPose();
	mVision.updateCameraPose(robotPose);
	mField.setRobotPose(robotPose);

	// put data to dashboard.
	SmartDashboard.putData(mField);
    }

}
