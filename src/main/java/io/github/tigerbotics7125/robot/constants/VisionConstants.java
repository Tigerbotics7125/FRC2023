/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {

    public static final String VISION_CAM_NAME = "OV5647";
    public static final String DRIVER_CAM_NAME = "HD_Pro_Webcam_C920";

    public static final Transform3d CAMERA_TO_ROBOT_TRANSFORM =
            new Transform3d(
                            new Translation3d(
                                    Units.inchesToMeters(17),
                                    Units.inchesToMeters(3),
                                    Units.inchesToMeters(9)),
                            new Rotation3d(0, 0, 0))
                    .inverse();

    // Sim variables
    public static final double DIAG_FOV_DEGREES = 67.0;

    public static final double MAX_DETECTION_DISTANCE = Units.feetToMeters(20);
    public static final int WIDTH_PIXELS = 640;
    public static final int HEIGHT_PIXELS = 480;
    public static final int MIN_TARGET_AREA = 10; // pixels^2
}
