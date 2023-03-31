/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.subsystem;

import static io.github.tigerbotics7125.robot.constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.tigerbotics7125.lib.ASUtil;
import io.github.tigerbotics7125.robot.Robot;
import io.github.tigerbotics7125.robot.constants.DashboardConstants;
import io.github.tigerbotics7125.robot.constants.FieldConstants;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Vision extends SubsystemBase {

    private PhotonCamera mLeftCam;
    private PhotonCamera mRightCam;

    private PhotonPoseEstimator mLeftEstimator;
    private PhotonPoseEstimator mRightEstimator;

    private Pose3d mRobotPose = new Pose3d();
    public double[] mLeftCamPose = new double[7];
    public double[] mRightCamPose = new double[7];

    public Vision() {

        mLeftCam = new PhotonCamera(LEFT_CAM_NAME);
        mRightCam = new PhotonCamera(RIGHT_CAM_NAME);
        mLeftCam.setDriverMode(false);
        mRightCam.setDriverMode(false);

        mLeftEstimator =
                new PhotonPoseEstimator(
                        FieldConstants.APRIL_TAG_FIELD_LAYOUT,
                        PoseStrategy.MULTI_TAG_PNP,
                        mLeftCam,
                        ROBOT_TO_LEFT_CAM_TRANSFORM);
        mRightEstimator =
                new PhotonPoseEstimator(
                        FieldConstants.APRIL_TAG_FIELD_LAYOUT,
                        PoseStrategy.MULTI_TAG_PNP,
                        mRightCam,
                        ROBOT_TO_RIGHT_CAM_TRANSORM);

        mLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        mRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        var tab = DashboardConstants.ODO_VISION_TAB;
        tab.addDoubleArray(
                "RobotPose",
                () -> ASUtil.pose3dToDoubleArray(new Pose3d(Robot.mDrivetrain.getPose())));
        tab.addDoubleArray(
                "Left Cam",
                () ->
                        ASUtil.pose3dToDoubleArray(
                                new Pose3d(Robot.mDrivetrain.getPose())
                                        .transformBy(ROBOT_TO_LEFT_CAM_TRANSFORM)));
        tab.addDoubleArray(
                "Right Cam",
                () ->
                        ASUtil.pose3dToDoubleArray(
                                new Pose3d(Robot.mDrivetrain.getPose())
                                        .transformBy(ROBOT_TO_RIGHT_CAM_TRANSORM)));
        tab.addDoubleArray("Left Cam Pose Estimate", () -> mLeftCamPose);
        tab.addDoubleArray("Right Cam Pose Estimate", () -> mRightCamPose);
    }

    public List<Optional<EstimatedRobotPose>> getEstimatedPoses() {
        Optional<EstimatedRobotPose> leftPose = mLeftEstimator.update();
        Optional<EstimatedRobotPose> rightPose = mRightEstimator.update();

        leftPose.ifPresent((x) -> mLeftCamPose = ASUtil.pose3dToDoubleArray(x.estimatedPose));
        rightPose.ifPresent((x) -> mRightCamPose = ASUtil.pose3dToDoubleArray(x.estimatedPose));

        return List.of(leftPose, rightPose);
    }

    @Override
    public void periodic() {
        getEstimatedPoses().forEach((pose) -> pose.ifPresent(Robot.mDrivetrain::addVisionEstimate));
        List<Pose2d> visionPoses =
                getEstimatedPoses().stream()
                        .filter(Optional::isPresent)
                        .map((p) -> p.get().estimatedPose.toPose2d())
                        .toList();
        Robot.mDrivetrain.mField.getObject("visionPoses").setPoses(visionPoses);
    }

    @Override
    public void simulationPeriodic() {}
}
