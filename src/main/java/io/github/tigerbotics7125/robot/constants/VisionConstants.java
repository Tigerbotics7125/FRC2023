/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonTargetSortMode;

public final class VisionConstants {

    public static final class Sim {
        public static final double kCamDiagFOVDegrees = 67.0;

        // also just max target detection distance.
        public static final double kMaxLEDRangeMeters = Units.feetToMeters(30);
        public static final int kCamResWidth = 640;
        public static final int kCamResHeight = 480;
        public static final int kMinTargetArea = 10; // pixels^2
    }

    public static final String kCameraName = "OV5647";

    public static final PhotonTargetSortMode kSortingMode = PhotonTargetSortMode.Largest;
    public static final double kAmbiguityThreshold = 0.2;

    public static final Transform3d kDefaultCamToRobot =
            new Transform3d(
                            /* Robot Pose */ new Pose3d(), /* Cam relative to robot */
                            new Pose3d(
                                    0,
                                    0,
                                    Units.feetToMeters(0.5),
                                    new Rotation3d(0, Units.degreesToRadians(10), 0)))
                    .inverse();
}
