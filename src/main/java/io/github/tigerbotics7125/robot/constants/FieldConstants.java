/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.List;

public class FieldConstants {

    public static final double LENGTH_METERS = 16.54175;
    public static final double WIDTH_METERS = 8.0137;

    public static final AprilTag TAG1 =
            new AprilTag(
                    1,
                    new Pose3d(
                            15.513558,
                            1.071626,
                            0.462788,
                            new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0))));
    public static final AprilTag TAG2 =
            new AprilTag(
                    2,
                    new Pose3d(
                            15.513558,
                            2.748026,
                            0.462788,
                            new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0))));
    public static final AprilTag TAG3 =
            new AprilTag(
                    3,
                    new Pose3d(
                            15.513558,
                            4.424426,
                            0.462788,
                            new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0))));
    public static final AprilTag TAG4 =
            new AprilTag(
                    4,
                    new Pose3d(
                            16.178784,
                            6.749796,
                            0.695452,
                            new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0))));
    public static final AprilTag TAG5 =
            new AprilTag(
                    5,
                    new Pose3d(
                            0.36195,
                            6.749796,
                            0.695452,
                            new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))));
    public static final AprilTag TAG6 =
            new AprilTag(
                    6,
                    new Pose3d(
                            1.02743,
                            4.424426,
                            0.462788,
                            new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))));
    public static final AprilTag TAG7 =
            new AprilTag(
                    7,
                    new Pose3d(
                            1.02743,
                            2.748026,
                            0.462788,
                            new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))));
    public static final AprilTag TAG8 =
            new AprilTag(
                    8,
                    new Pose3d(
                            1.02743,
                            1.071626,
                            0.462788,
                            new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))));

    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
            new AprilTagFieldLayout(
                    List.of(TAG1, TAG2, TAG3, TAG4, TAG5, TAG6, TAG7, TAG8),
                    LENGTH_METERS,
                    WIDTH_METERS);
}
