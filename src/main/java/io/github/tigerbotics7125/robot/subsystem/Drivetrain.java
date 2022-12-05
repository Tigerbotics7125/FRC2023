/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.subsystem;

import static io.github.tigerbotics7125.robot.Constants.Drivetrain.*;

import io.github.tigerbotics7125.robot.Robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.REVPhysicsSim;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Drivetrain extends SubsystemBase {
    final List<CANSparkMax> mMotors;
    final WPI_PigeonIMU mPigeon;

    final MecanumDriveKinematics mKinematics;
    final MecanumDrivePoseEstimator mPoseEstimator;

    final ProfiledPIDController mThetaPID = kThetaPIDController;

    boolean mFieldOriented = kFieldOrientedDefault;
    boolean mProtectHeading = kProtectHeadingDefault;
    Rotation2d mDesiredHeading = new Rotation2d();

    public Drivetrain() {
        mMotors =
                List.of(
                        new CANSparkMax(kFLID, kMotorType),
                        new CANSparkMax(kFRID, kMotorType),
                        new CANSparkMax(kRLID, kMotorType),
                        new CANSparkMax(kRRID, kMotorType));
        mMotors.forEach(this::initMotor);

        mPigeon = new WPI_PigeonIMU(kPigeonID);

        mKinematics = new MecanumDriveKinematics(kFLOffset, kFROffset, kRLOffset, kRROffset);
        mPoseEstimator =
                new MecanumDrivePoseEstimator(
                        getHeading(),
                        new Pose2d(),
                        mKinematics,
                        kStateStdDevs,
                        kLocalMeasurementStdDevs,
                        kVisionMeasurementStdDevs);
    }

    /** @param motor The motor to setup. */
    private void initMotor(CANSparkMax motor) {
        motor.setSmartCurrentLimit(kStallCurrentLimit, kFreeSpeedCurrentLimit);
        motor.setIdleMode(IdleMode.kCoast);
        motor.getPIDController().setP(.5);
        motor.getEncoder().setPositionConversionFactor(kPositionConversionFactor);
        motor.getEncoder().setVelocityConversionFactor(kVelocityConversionFactor);
        if (Robot.isSimulation()) REVPhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNEO(1));
    }

    @Override
    public void periodic() {
        mPoseEstimator.update(getHeading(), getWheelSpeeds());
    }

    @Override
    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().run();
    }

    /** Disables all motor output. */
    public void disable() {
        mMotors.forEach(CANSparkMax::disable);
    }

    public void resetGyro() {
        mPigeon.reset();
        mDesiredHeading = getHeading();
    }

    public void feedVisionTarget(PhotonTrackedTarget target) {
        // om nom nom

        if (target.getPoseAmbiguity() > kAmbiguityThreshold) return;
    }

    /**
     * @param xSpeed Speed in the forwards direction.
     * @param ySpeed Speed in the left direction.
     * @param zx X axis of the desired heading.
     * @param zy Y axis of the desired heading.
     */
    public void driveFaceAngle(double xSpeed, double ySpeed, double zx, double zy) {
        // This mode cannot be used while not field oriented.
        if (!mFieldOriented) {
            driveStandard(xSpeed, ySpeed, ySpeed);
            return;
        }
        if (!(zx == 0 && zy == 0)) {
            mDesiredHeading = new Rotation2d(zx, zy); // yay atan2 constructor!
        }
        SmartDashboard.putNumber("desiredHeading", mDesiredHeading.getRadians());
        SmartDashboard.putNumber("heading", Math.atan2(zx, zy));

        setWheelSpeeds(
                mKinematics.toWheelSpeeds(
                        new ChassisSpeeds(
                                xSpeed * kMaxTranslationVelocity,
                                ySpeed * kMaxTranslationVelocity,
                                /* PID is profiled, so no need to multiply to adjust speed */
                                mThetaPID.calculate(
                                        getHeading().getRadians(), mDesiredHeading.getRadians()))));
    }

    public void driveStandard(double xSpeed, double ySpeed, double zSpeed) {
        setWheelSpeeds(mKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, ySpeed, zSpeed)));
    }

    //
    // SETTERS
    //

    /** @param fieldOriented Whether to drive field oriented or robot oriented. */
    public void setFieldOriented(boolean fieldOriented) {
        mFieldOriented = fieldOriented;
    }

    /** @param protectHeading Wheter to attempt to maintain a heading while not turning. */
    public void setHeadingProtection(boolean protectHeading) {
        mProtectHeading = protectHeading;
    }

    /** Sets the idle mode to coast. */
    public void setCoastMode() {
        setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    /** Sets the idle mode to brake. */
    public void setBrakeMode() {
        setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    /** @param idleMode The idle mode to set the motors to. */
    public void setIdleMode(CANSparkMax.IdleMode idleMode) {
        mMotors.forEach((motor) -> motor.setIdleMode(idleMode));
    }

    /**
     * @param pose The new pose.
     * @param heading The new heading.
     */
    public void setPoseEstimation(Pose2d pose, Rotation2d heading) {
        mPoseEstimator.resetPosition(pose, heading);
        mPigeon.setFusedHeading(heading.getDegrees());
    }

    /** @param targetSpeeds The wheel speeds to have the robot attempt to achieve. */
    public void setWheelSpeeds(MecanumDriveWheelSpeeds targetSpeeds) {
        targetSpeeds.desaturate(kMaxTranslationVelocity);
        List<Double> wheelSpeeds =
                List.of(
                        targetSpeeds.frontLeftMetersPerSecond,
                        targetSpeeds.frontRightMetersPerSecond,
                        targetSpeeds.rearLeftMetersPerSecond,
                        targetSpeeds.rearRightMetersPerSecond);

        for (int i = 0; i < 4; i++) {
            mMotors.get(i)
                    .getPIDController()
                    .setReference(wheelSpeeds.get(i), ControlType.kVelocity);
        }
    }

    //
    // GETTERS
    //

    /** @return If the robot will protect its heading. */
    public boolean isProtectingHeading() {
        return mProtectHeading;
    }

    /** @return If the robot is driving field oriented. */
    public boolean isFieldOriented() {
        return mFieldOriented;
    }

    /** @return The current estimated robot position. */
    public Pose2d getPose() {
        return mPoseEstimator.getEstimatedPosition();
    }

    /** @return The robot's heading angle. */
    public Rotation2d getHeading() {
        return mPigeon.getRotation2d();
    }

    /** @return The current wheel speeds. */
    public MecanumDriveWheelSpeeds getWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(
                mMotors.get(0).getEncoder().getVelocity(),
                mMotors.get(1).getEncoder().getVelocity(),
                mMotors.get(2).getEncoder().getVelocity(),
                mMotors.get(3).getEncoder().getVelocity());
    }
}
