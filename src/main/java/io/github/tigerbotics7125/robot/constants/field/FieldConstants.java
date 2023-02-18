/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.constants.field;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;

public class FieldConstants {
    // setup in static block
    public static final AprilTagFieldLayout mTagLayout;

    static {
        AprilTagFieldLayout tagLayout;
        try {
            tagLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        } catch (IOException e) {
            DriverStation.reportError(
                    "Failed to instatiate AprilTagFieldLayout.", e.getStackTrace());
            tagLayout = null;
        }
        mTagLayout = tagLayout;
    }
}
