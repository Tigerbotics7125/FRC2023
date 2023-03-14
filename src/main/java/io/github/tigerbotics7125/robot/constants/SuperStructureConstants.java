/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.constants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;

public class SuperStructureConstants {
    public static final int ELEVATOR_HEIGHT_SENSOR_CHANNEL = 0;
    public static final AnalogInput ELEVATOR_HEIGHT_SENSOR_RAW = new AnalogInput(1);
    public static final int ELEV_MASTER_ID = 11;
    public static final int ELEV_FOLLOWER_ID = 12;

    public static final int ARM_ID = 21;

    public static final int WRIST_ID = 31;

    public static final MotorType MOTOR_TYPE = MotorType.kBrushless;

    public static final double ELEV_START_DIST = 0D;
    public static final double ELEV_MAX_DIST = 1.447800;
    public static final Rotation2d ELEV_ANGLE = new Rotation2d(Math.PI / 4D); // 45deg

    public static final double ARM_LENGTH = 0.457200;
    public static final Rotation2d ARM_START_ANGLE =
            new Rotation2d(Math.toRadians(10)); // 10 degrees inwards from pointing out of the arm.
    public static final Rotation2d ARM_MIN_ANGLE = ARM_START_ANGLE;
    public static final Rotation2d ARM_MAX_ANGLE =
            new Rotation2d(
                    Math.toRadians(
                            170)); // opposite direction of start angle, pointing back towards
    // robot.

    public static final double WRIST_LENGTH = 0.255694;
    public static final Rotation2d WRIST_START_ANGLE =
            new Rotation2d(Math.toRadians(180 + 35)); // pointing straight down to ground.
    public static final Rotation2d WRIST_MIN_ANGLE =
            new Rotation2d(Math.toRadians(180)); // pointing back towards base of arm.
    public static final Rotation2d WRIST_MAX_ANGLE =
            new Rotation2d(Math.toRadians(0)); // pointing away from arm.
}
