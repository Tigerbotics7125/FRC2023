/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.constants.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import io.github.tigerbotics7125.robot.AprilTagLayout;

/**
 * This enum represents different "zones" where the robot can autonomously allign to using vision
 * and odometry.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public enum FieldZone {
    DOUBLE_SUBSTATION_LEFT(
            AprilTagLayout.getTagPose(4)
                    .getTranslation()
                    .toTranslation2d()
                    .plus(new Translation2d(0, 0.721519))),
    DOUBLE_SUBSTATION_RIGHT(
            AprilTagLayout.getTagPose(4)
                    .getTranslation()
                    .toTranslation2d()
                    .plus(new Translation2d(0.0, -0.721519))),
    LEFT_GRID(
            AprilTagLayout.getTagPose(8)
                    .getTranslation()
                    .toTranslation2d()
                    .plus(new Translation2d(0.350520, 0.0))),
    CO_OP_GRID(
            AprilTagLayout.getTagPose(7)
                    .getTranslation()
                    .toTranslation2d()
                    .plus(new Translation2d(0.350520, 0.0))),
    RIGHT_GRID(
            AprilTagLayout.getTagPose(6)
                    .getTranslation()
                    .toTranslation2d()
                    .plus(new Translation2d(0.350520, 0.0)));

    private final Translation2d mPoseMeters;

    private FieldZone(Translation2d poseMeters) {
        mPoseMeters = poseMeters;
    }

    public Translation2d getTranslation() {
        Translation2d pose = mPoseMeters;
        if (!(DriverStation.getAlliance() == Alliance.Red)) return pose;
        // Flip for red alliance.

        if (this == DOUBLE_SUBSTATION_LEFT) {
            pose = DOUBLE_SUBSTATION_RIGHT.mPoseMeters;
        } else if (this == DOUBLE_SUBSTATION_RIGHT) {
            pose = DOUBLE_SUBSTATION_LEFT.mPoseMeters;
        }

        if (this == LEFT_GRID) {
            pose = RIGHT_GRID.mPoseMeters;
        } else if (this == RIGHT_GRID) {
            pose = LEFT_GRID.mPoseMeters;
        }

        pose = new Translation2d(FieldConstants.fieldLength - pose.getX(), pose.getY());
        return pose;
    }

    public Pose2d getPose() {
        return new Pose2d(getTranslation(), new Rotation2d());
    }
}
