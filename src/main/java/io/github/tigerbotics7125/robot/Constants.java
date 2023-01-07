/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import java.util.List;

import org.photonvision.PhotonTargetSortMode;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringTopic;
import io.github.tigerbotics7125.robot.AprilTagLayout.Layout;
import io.github.tigerbotics7125.robot.subsystem.Drivetrain.TurningMode;

public class Constants {

	public static final double kNominalVoltage = 12.0;

	public static class OperatorInterface {
		public static final int kDriverPort = 0;
		public static final int kOperatorPort = 1;
	}

	public static class NetworkTables {
		public static final NetworkTableInstance kNTInstance = NetworkTableInstance.getDefault();
		public static final NetworkTable kRobotTable = kNTInstance.getTable("robot");

		public static final StringTopic kTargetLockHeadingTopic = kRobotTable.getStringTopic("target_lock_degrees");

	}

	public static class Vision {
		public static final Layout kTagLayout = Layout.RAPID_REACT;
		public static final List<Integer> kTargetLockTags = List.of(40, 41, 42, 43, 50, 51, 52, 53);
		public static final PhotonTargetSortMode kSortingMode = PhotonTargetSortMode.Largest;
		public static final String kCameraName = "piCam";
		public static final double kCamDiagFOVDegrees = 67.0;
		public static final Transform3d kCamToRobot = new Transform3d(new Pose3d(), // robot
				new Pose3d(0, 0, .5, new Rotation3d(0, Units.degreesToRadians(20), 0))).inverse();
		public static final double kMaxLEDRangeMeters = Units.feetToMeters(30); // used
		// for
		// reflective.
		public static final int kCamResWidth = 640;
		public static final int kCamResHeight = 480;
		public static final int kMinTargetArea = 10; // pixels^2 // used for
		// reflective.

		public static final double kAmbiguityThreshold = 0.2;
	}

	public static class Drivetrain {
		// Driving Options
		public static final boolean kFieldOrientedDefault = true;
		public static final Rotation2d kDefaultHeading = new Rotation2d();
		public static final TurningMode kTurningModeDefault = TurningMode.JOYSTICK_ANGLE;

		// CAN IDs
		public static final int kFLID = 1;
		public static final int kRLID = 2;
		public static final int kFRID = 3;
		public static final int kRRID = 4;
		public static final int kPigeonID = 1;

		// Characteristics
		public static final double kMaxTranslationVelocity = Units.feetToMeters(13); // meters
		// /
		// second
		public static final double kMaxThetaVelocity = 4 * Math.PI; // radians /
		// seconds
		// reach max velocity in seconds. (a = v2-v1/t)
		public static final double kMaxThetaAcceleration = kMaxThetaVelocity / .25; // radians
		// /
		// seconds^2

		// PID
		public static final double kThetaPGain = 4; // proportional; effort =
		// error * p
		public static final double kThetaIGain = 0; // integral; effort = error
		// area * i
		public static final double kThetaDGain = .01; // derivative; effort =
		// de/dt * d
		public static final ProfiledPIDController kThetaPIDController = new ProfiledPIDController(kThetaPGain,
				kThetaIGain, kThetaDGain, new TrapezoidProfile.Constraints(kMaxThetaVelocity, kMaxThetaAcceleration));
		public static final double kWheelPGain = 1;
		public static final double kWheelIGain = 0;
		public static final double kWheelDGain = 0;

		// real world values
		public static final double kWheelRadiusMeters = Units.inchesToMeters(6.0);

		// Motor Values
		public static final MotorType kMotorType = MotorType.kBrushless;
		public static final int kStallCurrentLimit = 44; // Amps
		public static final int kFreeSpeedCurrentLimit = 2; // Amps
		public static final double kGearRatio = 8.45; // micro toughbox ratio
		/**
		 * Motor rotations to wheel meters.
		 * @formatter:off
		 *  1 revolution     gearbox output     2 * pi * wheel radius
		 * -------------- * ---------------- * -----------------------
		 *       1           gearbox input          1 revolution
		 * @formatter:on
		 */
		public static final double kPositionConversionFactor = 1.0 / kGearRatio * (2 * Math.PI * kWheelRadiusMeters);
		/**
		 * Input RPM to output RPM
		 * @formatter:off
		 *
		 * @formatter:on
		 */
		public static final double kVelocityConversionFactor = 1.0 / kGearRatio;

		// Kinematic Values
		public static final Translation2d kFLOffset = new Translation2d(Units.inchesToMeters(10.18),
				Units.inchesToMeters(-10.857));
		public static final Translation2d kRLOffset = new Translation2d(Units.inchesToMeters(-10.18),
				Units.inchesToMeters(-10.857));
		public static final Translation2d kFROffset = new Translation2d(Units.inchesToMeters(10.18),
				Units.inchesToMeters(10.857));
		public static final Translation2d kRROffset = new Translation2d(Units.inchesToMeters(-10.18),
				Units.inchesToMeters(10.857));

		// Pose Estimation Values
		// std devs, 0 is perfectly trusted, increase to trust less.
		// state, or pose estimator std devs [x, y, theta]
		public static final Matrix<N3, N1> kStateStdDevs = VecBuilder.fill(.1, .1, .1);
		// sensor (gyro and encoder) std devs
		public static final Matrix<N1, N1> kLocalMeasurementStdDevs = VecBuilder.fill(.05);
		// vision std devs [x, y, theta]
		public static final Matrix<N3, N1> kVisionMeasurementStdDevs = VecBuilder.fill(.075, .075, .075);
	}
}
