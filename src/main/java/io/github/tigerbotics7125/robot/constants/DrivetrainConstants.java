/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.constants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import io.github.tigerbotics7125.robot.subsystem.Drivetrain.TurningMode;

public final class DrivetrainConstants {

    // Default characteristics
    public static final boolean FIELD_ORIENTED_DEFAULT = true;
    public static final TurningMode TURNING_MODE_DEFAULT = TurningMode.JOYSTICK_ANGLE;
    public static final Rotation2d HEADING_DEFAULT = new Rotation2d();

    // CAN
    public static final int FRONT_LEFT_ID = 1;
    public static final int FRONT_RIGHT_ID = 2;
    public static final int REAR_LEFT_ID = 3;
    public static final int REAR_RIGHT_ID = 4;
    public static final int PIGEON_ID = 1;

    // Kinematics
    private static final double TRACK_WIDTH_METERS = 0.558145;
    private static final double TRACK_LENGTH_METERS = 0.517142;
    private static final Translation2d FRONT_LEFT_OFFSET =
            new Translation2d(TRACK_LENGTH_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0);
    private static final Translation2d FRONT_RIGHT_OFFSET =
            new Translation2d(TRACK_LENGTH_METERS / 2.0, TRACK_WIDTH_METERS / 2.0);
    private static final Translation2d REAR_LEFT_OFFSET =
            new Translation2d(-TRACK_LENGTH_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0);
    private static final Translation2d REAR_RIGHT_OFFSET =
            new Translation2d(-TRACK_LENGTH_METERS / 2.0, TRACK_WIDTH_METERS / 2.0);
    public static final MecanumDriveKinematics KINEMATICS =
            new MecanumDriveKinematics(
                    FRONT_LEFT_OFFSET, FRONT_RIGHT_OFFSET, REAR_LEFT_OFFSET, REAR_RIGHT_OFFSET);

    // Characteristics
    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(6.0) / 2.0;
    public static final double GEAR_RATIO = 10.71; // input

    // get from AM website
    public static final float STALL_TORQUE_NEWTON_METERS = 55.697f;
    public static final float FREE_SPEED_RPM = 529.97f; // RPM

    // meters / second
    public static final double MAX_LINEAR_VELOCITY_MPS = Units.feetToMeters(13.88);

    // radians / second
    public static final double MAX_ANGULAR_VELOCITY =
            MAX_LINEAR_VELOCITY_MPS / (TRACK_WIDTH_METERS / 2.0);

    // reach max velocity in seconds. (a = v2-v1/t)
    // radians / seconds^2
    public static final double MAX_ANGULAR_ACCELERATION = MAX_ANGULAR_VELOCITY / 2;

    public static final ProfiledPIDController THETA_PID_CONTROLLER;
    private static final double THETA_P_GAIN = 5.0;
    private static final double THETA_I_GAIN = 0.0;
    private static final double THETA_D_GAIN = 0.0;
    private static final Constraints THETA_CONSTRAINTS =
            new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION);
    private static final double THETA_TOLERANCE = Units.degreesToRadians(5);
    private static final double THETA_MIN_INPUT = 0.0;
    private static final double THETA_MAX_INPUT = 2.0 * Math.PI;

    static {
        THETA_PID_CONTROLLER =
                new ProfiledPIDController(
                        THETA_P_GAIN, THETA_I_GAIN, THETA_D_GAIN, THETA_CONSTRAINTS);
        THETA_PID_CONTROLLER.setTolerance(THETA_TOLERANCE);
        THETA_PID_CONTROLLER.enableContinuousInput(THETA_MIN_INPUT, THETA_MAX_INPUT);
    }

    public static final double WHEEL_P_GAIN = 0.1;
    public static final double WHEEL_I_GAIN = 0.0;
    public static final double WHEEL_D_GAIN = 0.0;

    // Motor values
    public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
    public static final int STALL_CURRENT_LIMIT_AMPS = 44;
    public static final int FREE_SPEED_CURRENT_LIMIT_AMPS = 5;
    public static final double POSITION_CONVERSION_FACTOR =
            (1.0 / GEAR_RATIO) * (2.0 * Math.PI * WHEEL_RADIUS_METERS);
    public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60.0;
}
