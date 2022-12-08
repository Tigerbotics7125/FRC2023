/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import io.github.tigerbotics7125.robot.subsystem.Drivetrain.TurningMode;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Constants {
    public static class OperatorInterface {
        public static final int kDriverPort = 0;
        public static final int kOperatorPort = 1;
    }

    public static class Vision {
        public static final double kAmbiguityThreshold = 0.2;
    }

    public static class Drivetrain {
        // Driving Options
        public static final boolean kFieldOrientedDefault = true;
        public static final Rotation2d kDefaultHeading = new Rotation2d();

        public static final TurningMode kTurningModeDefault = TurningMode.FACE_ANGLE;

        // CAN IDs
        public static final int kFLID = 1;
        public static final int kRLID = 2;
        public static final int kFRID = 3;
        public static final int kRRID = 4;
        public static final int kPigeonID = 1;

        // Characteristics
        public static final double kMaxTranslationVelocity =
                Units.feetToMeters(15); // meters / second
        public static final double kMaxThetaVelocity = 4 * Math.PI; // radians / seconds
        public static final double kMaxThetaAcceleration = 8 * Math.PI; // radians / seconds^2

        // PID
        public static final double kThetaPGain = 3; // proportional; effort = error * p
        public static final double kThetaIGain = 0; // integral; effort = error area * i
        public static final double kThetaDGain = .001; // derivative;  effort = de/dt * d
        public static final ProfiledPIDController kThetaPIDController =
                new ProfiledPIDController(
                        kThetaPGain,
                        kThetaIGain,
                        kThetaDGain,
                        new TrapezoidProfile.Constraints(kMaxThetaVelocity, kMaxThetaAcceleration));
        public static final double kWheelPGain = 1;
        public static final double kWheelIGain = 0;
        public static final double kWheelDGain = 0;

        // Motor Values
        public static final MotorType kMotorType = MotorType.kBrushless;
        public static final int kStallCurrentLimit = 44; // Amps
        public static final int kFreeSpeedCurrentLimit = 2; // Amps
        public static final double kGearRatio = 8.45; // micro toughbox ratio
        public static final double kPositionConversionFactor =
                1.0 / kGearRatio; // input rotations to output rotations.
        public static final double kVelocityConversionFactor =
                1.0 / kGearRatio; // input RPM to output RPM.

        // Kinematic Values
        public static final Translation2d kFLOffset =
                new Translation2d(Units.inchesToMeters(10.18), Units.inchesToMeters(-10.857));
        public static final Translation2d kRLOffset =
                new Translation2d(Units.inchesToMeters(-10.18), Units.inchesToMeters(-10.857));
        public static final Translation2d kFROffset =
                new Translation2d(Units.inchesToMeters(10.18), Units.inchesToMeters(10.857));
        public static final Translation2d kRROffset =
                new Translation2d(Units.inchesToMeters(-10.18), Units.inchesToMeters(10.857));

        // Pose Estimation Values
        // std devs, 0 is perfectly trusted, increase to trust less.
        // state, or pose estimator std devs [x, y, theta]
        public static final Matrix<N3, N1> kStateStdDevs = VecBuilder.fill(.001, .001, .001);
        // sensor (gyro and encoder) std devs
        public static final Matrix<N1, N1> kLocalMeasurementStdDevs = VecBuilder.fill(.01);
        // vision std devs [x, y, theta]
        public static final Matrix<N3, N1> kVisionMeasurementStdDevs =
                VecBuilder.fill(.1, .1, .125);
    }
}
