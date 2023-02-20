/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import io.github.tigerbotics7125.robot.constants.field.FieldZone;
import io.github.tigerbotics7125.robot.constants.field.GridLocation;

/**
 * This class stores all measurement constants as the blue alliance and flips them upon retrieving
 * if the alliance given by driverstation is red.
 */
public class AutoPilotConstants {

    public static final double MAX_VELOCITY_MPS = 5;
    public static final double MAX_ACCELERATION_MPSPS = 4;

    public static final PIDController X_PID = new PIDController(1, 0, 0);
    public static final PIDController Y_PID = new PIDController(1, 0, 0);
    public static final PIDController THETA_PID = new PIDController(1, 0, 0);

    public static Pose2d getAutoPilotPose(FieldZone zone, GridLocation.Column gridPosition) {

        Rotation2d rotation;
        Translation2d pose;

        // Get rotation, substation faces away from you so zero, grid is towards you so 180.
        if (zone == FieldZone.DOUBLE_SUBSTATION_LEFT || zone == FieldZone.DOUBLE_SUBSTATION_RIGHT) {
            rotation = new Rotation2d();
        } else {
            rotation = Rotation2d.fromDegrees(180);
        }

        pose = zone.getTranslation();

        // add grid transform if the zone is a grid zone
        if (!(zone == FieldZone.DOUBLE_SUBSTATION_LEFT
                || zone == FieldZone.DOUBLE_SUBSTATION_RIGHT)) {
            pose = pose.plus(gridPosition.getPose2d());
        }

        return new Pose2d(pose, rotation);
    }
}
