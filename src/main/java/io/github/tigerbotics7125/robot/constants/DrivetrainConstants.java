/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.constants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import io.github.tigerbotics7125.robot.subsystem.Drivetrain.TurningMode;

public final class DrivetrainConstants {

    public static final class DefaultDrivingOptions {
        public static final boolean kFieldOriented = true;
        public static final TurningMode kTurningMode = TurningMode.JOYSTICK_ANGLE;
        public static final Rotation2d kDefaultHeading = new Rotation2d();
    }

    public static final class CAN {
        public static final int kFLID = 1;
        public static final int kFRID = 2;
        public static final int kRLID = 3;
        public static final int kRRID = 4;
        public static final int kPigeonID = 1;
    }

    public static final class Characteristics {
        public static final double kWheelRadiusMeters = Units.inchesToMeters(6.0);
        public static final double kGearRatio = 1.0 / 10.71;

        // meters / second
        public static final double kMaxLinearVelocity = Units.feetToMeters(15.6);
        // radians / second
        public static final double kMaxRotationalVelocity = 3.0 * Math.PI / 2.0;
        // reach max velocity in seconds. (a = v2-v1/t)
        // radians / seconds^2
        public static final double kMaxRotationalAcceleration = kMaxRotationalVelocity / .25;

        public static final ProfiledPIDController kThetaPIDController =
                new ProfiledPIDController(
                        5,
                        0,
                        .2,
                        new TrapezoidProfile.Constraints(
                                kMaxRotationalVelocity, kMaxRotationalAcceleration));
        // radians within goal to stop theta pid.
        private static final double kThetaPIDCutoff = Units.degreesToRadians(1);

        public static final double kWheelPGain = 1;
        public static final double kWheelIGain = 0;
        public static final double kWheelDGain = 0;

        static {
            kThetaPIDController.setTolerance(kThetaPIDCutoff);
        }
    }

    public static final class MotorValues {
        public static final MotorType kMotorType = MotorType.kBrushless;
        public static final int kStallCurrentLimit = 44; // Amps
        public static final int kFreeSpeedCurrentLimit = 2; // Amps
        /**
         * Motor rotations to wheel meters.
         *
         * @formatter:off 1 revolution gearbox output 2 * pi * wheel radius -------------- *
         *     ---------------- * ----------------------- 1 gearbox input 1 revolution
         * @formatter:on
         */
        public static final double kPositionConversionFactor =
                (1.0 / Characteristics.kGearRatio)
                        * (2 * Math.PI * Characteristics.kWheelRadiusMeters);
        /**
         * Input RPM to output meters per second
         *
         * @formatter:off 1 revolution gearbox output 2 * pi * wheel radius 1 minute --------------
         *     * ---------------- * ----------------------- * ------------ 1 minute gearbox input 1
         *     revolution 60 seconds
         * @formatter:on
         */
        public static final double kVelocityConversionFactor =
                (1.0 / Characteristics.kGearRatio)
                        * (2 * Math.PI * Characteristics.kWheelRadiusMeters)
                        * (1.0 / 60.0);
    }

    public static final class Kinematics {
        public static final Translation2d kFLOffset =
                new Translation2d(Units.inchesToMeters(10.18), Units.inchesToMeters(-10.857));
        public static final Translation2d kRLOffset =
                new Translation2d(Units.inchesToMeters(-10.18), Units.inchesToMeters(-10.857));
        public static final Translation2d kFROffset =
                new Translation2d(Units.inchesToMeters(10.18), Units.inchesToMeters(10.857));
        public static final Translation2d kRROffset =
                new Translation2d(Units.inchesToMeters(-10.18), Units.inchesToMeters(10.857));
    }

    public static final class Odometry {
        // Pose Estimation Values
        // std devs, 0 is perfectly trusted, increase to trust less.
        // state, or pose estimator std devs [x, y, theta]
        public static final Matrix<N3, N1> kStateStdDevs = VecBuilder.fill(.1, .1, .1);
        // sensor (gyro and encoder) std devs
        public static final Matrix<N1, N1> kLocalMeasurementStdDevs = VecBuilder.fill(.05);
        // vision std devs [x, y, theta]
        public static final Matrix<N3, N1> kVisionMeasurementStdDevs =
                VecBuilder.fill(.075, .075, .075);
    }
}
