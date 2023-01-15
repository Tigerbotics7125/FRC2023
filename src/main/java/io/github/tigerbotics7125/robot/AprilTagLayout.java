/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import java.util.Collection;
import java.util.Map;

/** Class used to house info for an entire field of AprilTags. */
public class AprilTagLayout {

    // contains the tag objects mapped to their fiducial id.
    public static Map<Integer, Pose3d> mTags =
            Map.of(
                    1,
                    new Pose3d(
                            Units.inchesToMeters(610.77),
                            Units.inchesToMeters(42.19),
                            Units.inchesToMeters(18.22),
                            new Rotation3d(0.0, 0.0, Math.PI)),
                    2,
                    new Pose3d(
                            Units.inchesToMeters(610.77),
                            Units.inchesToMeters(108.19),
                            Units.inchesToMeters(18.22),
                            new Rotation3d(0.0, 0.0, Math.PI)),
                    3,
                    new Pose3d(
                            Units.inchesToMeters(610.77),
                            Units.inchesToMeters(174.19),
                            Units.inchesToMeters(18.22),
                            new Rotation3d(0.0, 0.0, Math.PI)),
                    4,
                    new Pose3d(
                            Units.inchesToMeters(636.96),
                            Units.inchesToMeters(265.74),
                            Units.inchesToMeters(27.38),
                            new Rotation3d(0.0, 0.0, Math.PI)),
                    5,
                    new Pose3d(
                            Units.inchesToMeters(14.25),
                            Units.inchesToMeters(265.74),
                            Units.inchesToMeters(27.38),
                            new Rotation3d()),
                    6,
                    new Pose3d(
                            Units.inchesToMeters(40.45),
                            Units.inchesToMeters(174.19),
                            Units.inchesToMeters(18.22),
                            new Rotation3d()),
                    7,
                    new Pose3d(
                            Units.inchesToMeters(40.45),
                            Units.inchesToMeters(108.19),
                            Units.inchesToMeters(18.22),
                            new Rotation3d()),
                    8,
                    new Pose3d(
                            Units.inchesToMeters(40.45),
                            Units.inchesToMeters(42.19),
                            Units.inchesToMeters(18.22),
                            new Rotation3d()));

    /** @return All tags housed in this layout. */
    public static Collection<Pose3d> getTags() {
        return mTags.values();
    }

    /**
     * @param tagID The tag id to lookup.
     * @return the 3d position of that tag.
     */
    public static Pose3d getTagPose(int tagID) {
        return mTags.get(tagID);
    }
}
