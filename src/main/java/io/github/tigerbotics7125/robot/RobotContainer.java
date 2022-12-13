/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import static io.github.tigerbotics7125.robot.Constants.Drivetrain.kTargetLockTags;
import static io.github.tigerbotics7125.robot.Constants.OperatorInterface.kDriverPort;
import static io.github.tigerbotics7125.robot.Constants.OperatorInterface.kOperatorPort;
import static io.github.tigerbotics7125.robot.Constants.Vision.kAmbiguityThreshold;
import static io.github.tigerbotics7125.robot.Constants.Vision.kSortingMode;

import io.github.tigerbotics7125.robot.subsystem.Drivetrain;
import io.github.tigerbotics7125.robot.subsystem.Vision;
import io.github.tigerbotics7125.tigerlib.input.controller.XboxController;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger.ActivationCondition;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class RobotContainer {

    XboxController mDriver = new XboxController(kDriverPort);
    XboxController mOperator = new XboxController(kOperatorPort);
    Trigger mRioUserButton = new Trigger(RobotController::getUserButton);

    Drivetrain mDrivetrain = new Drivetrain();
    Vision mVision = new Vision();

    Field2d mField = new Field2d();

    boolean correcting = false;

    public RobotContainer() {

        initDriver();
        initOperator();

        initTriggers();

        initDefaultCommands();

        if (Robot.isSimulation()) {
            List<Pose2d> tagPoses = new ArrayList<>();
            mVision.getTagLayout().getTags().forEach((t) -> tagPoses.add(t.getPose().toPose2d()));
            mField.getObject("16h5Tags").setPoses(tagPoses);
        }
    }

    /** Initialize Driver Triggers. */
    private void initDriver() {
        mDriver.start().trigger(mDrivetrain::resetGyro);
        mDriver.lb().trigger(mDrivetrain::setStandardTurning);
        mDriver.y().trigger(mDrivetrain::setHoldTurning);
        mDriver.rb().trigger(mDrivetrain::setFaceAngleTurning);

        mDriver.back().trigger(() -> correcting = true);
        mDriver.back().activate(ActivationCondition.ON_FALLING).trigger(() -> correcting = false);
    }

    /** Initialize Operator Triggers. */
    private void initOperator() {}

    /** Initialize non-OI Triggers. */
    private void initTriggers() {
        new Trigger(RobotController::getUserButton)
                .trigger(() -> mDrivetrain.setPose(new Pose2d(), new Rotation2d()));
        new Trigger(RobotState::isDisabled).trigger(mDrivetrain::setCoastMode);
        new Trigger(RobotState::isEnabled).trigger(mDrivetrain::setBrakeMode);
    }

    /** Set Subsystem's default commands. */
    private void initDefaultCommands() {
        mDrivetrain.setDefaultCommand(
                mDrivetrain.drive(
                        mDriver.leftY()::get,
                        mDriver.leftX()::get,
                        mDriver.rightX()::get,
                        mDriver.rightY()::get));
    }

    /** Periodic call, always runs. */
    public void periodic() {
        visionLogic();
    }

    /** Periodic call, only runs during simulation. */
    public void simulationPeriodic() {
        // update sim vision with our robot pose.
        Pose2d robotPose = mDrivetrain.getPose();
        mVision.feedRobotPose(robotPose);

        mField.setRobotPose(mDrivetrain.getPose());

        // put data to dashboard.
        SmartDashboard.putData(mField);
    }

    public void visionLogic() {
        // Current vision result.
        PhotonPipelineResult result = mVision.getLatestResult();

        if (!result.hasTargets()) {
            mDrivetrain.disableTargetLock();
            return;
        }

        List<PhotonTrackedTarget> targets = result.getTargets();

        // remove tags which are too ambiguous
        List<PhotonTrackedTarget> ambiguousTags = new ArrayList<>();
        targets.forEach(
                (t) -> {
                    if (t.getPoseAmbiguity() > kAmbiguityThreshold) {
                        ambiguousTags.add(t);
                    }
                });
        ambiguousTags.forEach(targets::remove);

        // Sort the targets in our predefined desired manner.
        targets.sort(kSortingMode.getComparator());

        // Best target will be the first target now that it is sorted.
        var bestTarget = targets.get(0);

        /**
         * Givin a game like RAPID REACT, you should really be averaging a pose between all of the
         * seen hub tags, so that you would not shoot over the side of the hub.
         */

        // Set drivetrain target lock.
        if (kTargetLockTags.contains(bestTarget.getFiducialId())) {
            Pose2d tagPose =
                    mVision.getTagLayout().getTagPose(bestTarget.getFiducialId()).toPose2d();
            Pose2d robotPose = mDrivetrain.getPose();

            Transform2d robotToTag = new Transform2d(robotPose, tagPose);

            // Law of sines used to find the rotation needed to face the target.
            Rotation2d faceTarget =
                    new Rotation2d(robotToTag.getY() * Math.sin(Math.PI / 2) / robotToTag.getX());

            mDrivetrain.enableTargetLock(robotPose.getRotation().rotateBy(faceTarget));
            mField.getObject("targetlock").setPose(tagPose);
        }

        // Update drivetrain odometry with pose derived from target(s).
        if (correcting) {
            for (PhotonTrackedTarget target : targets) {
                mDrivetrain.addVisionMeasurement(
                        mVision.getRobotPoseFromTarget(target).toPose2d(),
                        result.getTimestampSeconds());
            }
        }

        if (Robot.isSimulation()) {
            // lists used to house the different poses.
            List<Pose2d> bestPoses = new ArrayList<>();
            List<Pose2d> altPoses = new ArrayList<>();
            List<Pose2d> seenTagPoses = new ArrayList<>();

            // determine the best, and alternate pose for every target.
            for (var target : result.getTargets()) {
                Pose3d tagPose = mVision.getTagLayout().getTagPose(target.getFiducialId());
                Transform3d bestTagToRobot = target.getBestCameraToTarget().inverse();
                Transform3d altTagToRobot = target.getAlternateCameraToTarget().inverse();

                bestPoses.add(tagPose.transformBy(bestTagToRobot).toPose2d());
                altPoses.add(tagPose.transformBy(altTagToRobot).toPose2d());
                seenTagPoses.add(tagPose.toPose2d());
            }

            // put all determined poses on the field object.
            mField.getObject("bestPoses").setPoses(bestPoses);
            mField.getObject("altPoses").setPoses(altPoses);
            mField.getObject("seenTags").setPoses(seenTagPoses);
            mField.getObject("bestTag")
                    .setPose(
                            mVision.getTagLayout()
                                    .getTagPose(bestTarget.getFiducialId())
                                    .toPose2d());
        }
    }
}
