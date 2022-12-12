/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.subsystem;

import static io.github.tigerbotics7125.robot.Constants.Vision.*;

import io.github.tigerbotics7125.robot.AprilTagLayout;
import io.github.tigerbotics7125.robot.AprilTagLayout.AprilTag;
import io.github.tigerbotics7125.robot.Robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

    PhotonCamera mCam = new PhotonCamera(kCameraName);
    AprilTagLayout mTagLayout = new AprilTagLayout(kTagLayout);

    SimVisionSystem mSimVision;

    public Vision() {
        // use camera for vision tracking, not for driver vision.
        mCam.setDriverMode(false);

        if (Robot.isSimulation()) {
            // Create a sim camera system.
            mSimVision =
                    new SimVisionSystem(
                            kCameraName,
                            kCamDiagFOVDegrees,
                            kCamToRobot,
                            kMaxLEDRangeMeters,
                            kCamResWidth,
                            kCamResHeight,
                            kMinTargetArea);
            // Add targets to simulation.
            for (AprilTag tag : mTagLayout.getTags()) {
                mSimVision.addSimVisionTarget(
                        new SimVisionTarget(
                                tag.getPose(),
                                Units.inchesToMeters(6),
                                Units.inchesToMeters(6),
                                tag.getID()));
            }
        }
    }

    /** @return The {@link AprilTagLayout} used by the robot. */
    public AprilTagLayout getTagLayout() {
        return mTagLayout;
    }

    /** @return The latest result from the vision system. */
    public PhotonPipelineResult getLatestResult() {
        return mCam.getLatestResult();
    }

    /** SIM ONLY call to update the sim vision system. */
    public void feedRobotPose(Pose2d robotPose) {
        if (Robot.isReal()) return;
        mSimVision.processFrame(robotPose);
    }

    /**
     * @param target The target to use to calculate the robot's position.
     * @return The robots pose as known by the vision system.
     */
    public Pose3d getRobotPoseFromTarget(PhotonTrackedTarget target) {
        Transform3d targetToCam = target.getBestCameraToTarget().inverse();

        Pose3d robotPose =
                mTagLayout
                        .getTagPose(target.getFiducialId())
                        .transformBy(targetToCam)
                        .transformBy(kCamToRobot);
        return robotPose;
    }
}
