/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import static io.github.tigerbotics7125.robot.constants.OIConstants.*;
import static io.github.tigerbotics7125.robot.constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import io.github.tigerbotics7125.robot.commands.AutoPilot;
import io.github.tigerbotics7125.robot.constants.field.FieldArea;
import io.github.tigerbotics7125.robot.subsystem.Drivetrain;
import io.github.tigerbotics7125.robot.subsystem.Drivetrain.TurningMode;
import io.github.tigerbotics7125.robot.subsystem.Vision;
import io.github.tigerbotics7125.tigerlib.input.controller.XboxController;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger.ActivationCondition;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;

public class RobotContainer {

    XboxController mDriver = new XboxController(kDriverPort);
    XboxController mOperator = new XboxController(kOperatorPort);
    Trigger mRioUserButton = new Trigger(RobotController::getUserButton);

    Drivetrain mDrivetrain = new Drivetrain();
    Vision mVision = new Vision();

    // Command
    AutoPilot mAutoPilot = new AutoPilot(mDrivetrain);

    Field2d mField = new Field2d();

    boolean correcting = false;

    public RobotContainer() {

        initDriver();
        initOperator();

        initTriggers();

        initDefaultCommands();

        if (Robot.isSimulation()) {
            mField.getObject("AprilTags")
                    .setPoses(AprilTagLayout.getTags().stream().map(Pose3d::toPose2d).toList());
        }
    }

    /** Initialize Driver Triggers. */
    private void initDriver() {
        mDriver.b().trigger(mDrivetrain::resetGyro);
        mDriver.lb().trigger(() -> mDrivetrain.setTurningMode(TurningMode.JOYSTICK_DIRECT));
        mDriver.rb().trigger(() -> mDrivetrain.setTurningMode(TurningMode.JOYSTICK_ANGLE));
        mDriver.y().trigger(() -> mDrivetrain.setTurningMode(TurningMode.HEADING_LOCK));

        mDriver.start().debounce(.02).activate(ActivationCondition.WHILE_HIGH).trigger(mAutoPilot);

        mDriver.back().trigger(() -> correcting = true);
        mDriver.back().activate(ActivationCondition.ON_FALLING).trigger(() -> correcting = false);
    }

    /** Initialize Operator Triggers. */
    private void initOperator() {}

    /** Initialize non-OI Triggers. */
    private void initTriggers() {
        new Trigger(RobotController::getUserButton)
                .trigger(() -> mDrivetrain.setPose(new Pose2d(), new Rotation2d()));
        new Trigger(RobotState::isDisabled).trigger(Commands.run(mDrivetrain::setCoastMode).ignoringDisable(true));
        new Trigger(RobotState::isEnabled).trigger(mDrivetrain::setBrakeMode);

        new Trigger(mVision::hasTargets)
                .trigger(
                        () -> {
                            double timestamp = mVision.getTimestamp();
                            mVision.getRobotPoseEstimates(kAmbiguityThreshold)
                                    .forEach(
                                            (pose) -> {
                                                mDrivetrain.addVisionMeasurement(
                                                        pose.toPose2d(), timestamp);
                                            });
                        });
    }

    /** Set Subsystem's default commands. */
    private void initDefaultCommands() {
        mDrivetrain.setDefaultCommand(
                Commands.run(
                        () ->
                                mDrivetrain.drive(
                                        mDriver.leftY().get(),
                                        mDriver.leftX().get(),
                                        mDriver.rightX().get(),
                                        mDriver.rightY().get()),
                        mDrivetrain));
    }

    /** Periodic call, always runs. */
    public void periodic() {
        mAutoPilot.generatePath();

        mField.setRobotPose(mDrivetrain.getPose());
        SmartDashboard.putData(mField);

        List<Pose2d> trajPoses = new ArrayList<>();
        // DS always on trajectory
        mAutoPilot.getTrajectory().getStates().forEach((state) -> trajPoses.add(state.poseMeters));
        mField.getObject("AutoPilot_GenPath").setPoses(trajPoses);
        // Active path.
        trajPoses.clear();
        mAutoPilot
                .getActiveTrajectory()
                .getStates()
                .forEach((state) -> trajPoses.add(state.poseMeters));
        mField.getObject("AutoPilot_ActivePath").setPoses(trajPoses);

        mField.getObject("CHARGING_STATION").setPoses(FieldArea.CHARGING_STATION.getPoses());
        mField.getObject("LOADING_ZONE").setPoses(FieldArea.LOADING_ZONE.getPoses());
        mField.getObject("COMMUNITY").setPoses(FieldArea.COMMUNITY.getPoses());

        SmartDashboard.putBoolean(
                "WithinCommunity", FieldArea.COMMUNITY.contains(mDrivetrain.getPose()));
    }

    /** Periodic call, only runs during simulation. */
    public void simulationPeriodic() {
        // update sim vision with our robot pose.
        mVision.updateCameraPose(mDrivetrain.getPose());

        List<PhotonTrackedTarget> targets = new ArrayList<>(mVision.getTargets());
        if (targets.size() > 0) {
            mField.getObject("SeenTags")
                    .setPoses(
                            targets.stream()
                                    .map(
                                            (target) ->
                                                    AprilTagLayout.getTagPose(
                                                                    target.getFiducialId())
                                                            .toPose2d())
                                    .toList());
            mField.getObject("BestTag")
                    .setPose(
                            AprilTagLayout.getTagPose(
                                            mVision.getBestTag(kSortingMode, kAmbiguityThreshold)
                                                    .getFiducialId())
                                    .toPose2d());
        }
    }
}
