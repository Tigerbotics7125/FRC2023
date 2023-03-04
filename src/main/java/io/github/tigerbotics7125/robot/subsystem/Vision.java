/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.subsystem;

import static io.github.tigerbotics7125.robot.constants.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import io.github.tigerbotics7125.robot.Robot;
import io.github.tigerbotics7125.robot.constants.FieldConstants;
import io.github.tigerbotics7125.tigerlib.vision.SnakeEyes;
import java.util.Objects;
import java.util.Optional;
import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Vision extends SnakeEyes implements Subsystem {

    private AprilTagFieldLayout mFieldLayout;
    private PhotonPoseEstimator mPhotonEstimator;

    private ShuffleboardTab mSBTab;
    private Field2d mDbgField;
    private FieldObject2d mKnownVisionTargets;
    private FieldObject2d mSeenVisionTargets;
    private FieldObject2d mEstimatedRobotPosition;

    private SimPhotonCamera mSimCam;
    private SimVisionSystem mSimVision;

    public Vision() {
        super(VISION_CAM_NAME, CAMERA_TO_ROBOT_TRANSFORM.inverse());
        // Register command to use periodic calls.
        register();

        // Cache result to prevent NPE.
        getLatestResult();

        // Initialize dashboard.
        mSBTab = Shuffleboard.getTab("Vision");

        mDbgField = new Field2d();
        mKnownVisionTargets = mDbgField.getObject("Known Vision Targets");
        mSeenVisionTargets = mDbgField.getObject("Seen Vision Targets");
        mEstimatedRobotPosition = mDbgField.getObject("Estimated Robot Pose");

        mSBTab.add(mDbgField);

        mFieldLayout = FieldConstants.APRIL_TAG_FIELD_LAYOUT;
        if (mFieldLayout != null) {
            mPhotonEstimator =
                    new PhotonPoseEstimator(
                            mFieldLayout,
                            PoseStrategy.MULTI_TAG_PNP,
                            this.mCam,
                            this.mRobotToCamera);
            mPhotonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }

        if (mFieldLayout != null) {
            mKnownVisionTargets.setPoses(
                    mFieldLayout.getTags().stream().map(tag -> tag.pose.toPose2d()).toList());
        }

        if (Robot.isSimulation()) {
            mSimVision =
                    new SimVisionSystem(
                            VISION_CAM_NAME,
                            DIAG_FOV_DEGREES,
                            CAMERA_TO_ROBOT_TRANSFORM,
                            MAX_DETECTION_DISTANCE,
                            WIDTH_PIXELS,
                            HEIGHT_PIXELS,
                            MIN_TARGET_AREA);
            mSimCam = new SimPhotonCamera(VISION_CAM_NAME);

            if (mFieldLayout != null) {
                // Add tags to simulation.
                mFieldLayout
                        .getTags()
                        .forEach(
                                tag ->
                                        mSimVision.addSimVisionTarget(
                                                new SimVisionTarget(
                                                        tag.pose,
                                                        Units.inchesToMeters(8),
                                                        Units.inchesToMeters(8),
                                                        tag.ID)));
            }
        }
    }

    /** @return The estimated pose as seen by the vision system. */
    public Optional<EstimatedRobotPose> getEstimatedPose() {
        // Field layout failed to load.
        if (mPhotonEstimator == null) return Optional.empty();

        Optional<EstimatedRobotPose> estimatedPose = mPhotonEstimator.update(mCachedResult);

        estimatedPose.ifPresent(
                pose -> mEstimatedRobotPosition.setPose(pose.estimatedPose.toPose2d()));

        return estimatedPose;
    }

    /**
     * Update the vision simulation.
     *
     * @param robotPose the robots pose.
     */
    public void update(Pose2d robotPose) {
        mSimVision.processFrame(robotPose);
    }

    @Override
    public void periodic() {
        // cache latest result
        this.getLatestResult();

        mSeenVisionTargets.setPoses(
                getTargets().stream()
                        .map(
                                (ptt) -> {
                                    if (mFieldLayout == null) {
                                        return null;
                                    }
                                    var tar = mFieldLayout.getTagPose(ptt.getFiducialId());
                                    if (tar.isPresent()) return tar.get().toPose2d();
                                    else return null;
                                })
                        .filter(o -> !Objects.isNull(o))
                        .toList());
    }

    @Override
    public void simulationPeriodic() {}
}
