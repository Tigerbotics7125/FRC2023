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
import io.github.tigerbotics7125.tigerlib.math.Conversion;

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
        public static final double kWheelRadiusMeters = Units.inchesToMeters(6.0) / 2.0;
        public static final double kGearRatio = 10.71; // input
        // get from AM website
        public static final float kStallTorque = 55.697001316f; // NM
        public static final float kFreeSpeed = 529.97f; // RPM

        // meters / second
        public static final double kMaxLinearVelocity = Units.feetToMeters(13.88);
        // radians / second
        public static final double kMaxRotationalVelocity = 1.5 * Math.PI / 2.0;
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

        public static final double kWheelPGain = 1.0e-6;
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
        // Converts FROM motor rotations TO wheel tangent meters
        public static final Conversion<Double, Double> kMotorsRotationsToWheelMeters =
                Conversion.create(
                        (motorRotations) -> {
                            double gearRatio = Characteristics.kGearRatio;
                            double wheelRadius = Characteristics.kWheelRadiusMeters;

                            // output rotations at gearbox output.
                            double wheelRotations = motorRotations / gearRatio;
                            // distance wheel has rotated tangentialy in meters
                            double wheelMeters = wheelRotations * wheelRadius;
                            return wheelMeters;
                        },
                        (wheelMeters) -> {
                            double gearRatio = Characteristics.kGearRatio;
                            double wheelRadius = Characteristics.kWheelRadiusMeters;

                            // output rotations at gearbox output.
                            double wheelRotations = wheelMeters / wheelRadius;
                            // motor rotations at gearbox input.
                            double motorRotations = wheelRotations * gearRatio;
                            return motorRotations;
                        });
        // Converts FROM motor RPM TO wheel tangential mps
        public static final Conversion<Double, Double> kMotorRPMToWheelMPS =
                Conversion.create(
                        (motorRPM) -> {
                            // convert rotations to meters
                            double wheelMetersPerMinute =
                                    kMotorsRotationsToWheelMeters.fromInput(motorRPM);
                            // convert minutes to seconds.
                            double wheelMetersPerSecond = wheelMetersPerMinute / 60.0;
                            return wheelMetersPerSecond;
                        },
                        (wheelMPS) -> {
                            // convert meters to rotations
                            double motorRotationsPerSecond =
                                    kMotorsRotationsToWheelMeters.fromOutput(wheelMPS);
                            // convert seconds to minutes;
                            double motorRotationsPerMinute = motorRotationsPerSecond * 60.0;
                            return motorRotationsPerMinute;
                        });
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