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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.REVPhysicsSim;
import java.util.List;
import java.util.function.DoubleSupplier;

public class Drivetrain extends SubsystemBase {
    public enum TurningMode {
        /**
         * Standard concept of turning, you push left on the joystick, the bot rotates left until
         * you return to center.
         */
        STANDARD,
        /**
         * Holds the current angle. Can be mildly dangerous if the robot is forced extremely outside
         * of its desired heading.
         */
        HOLD,
        /**
         * As opposed to typical turning, this mode is designed to follow your joystick, for
         * instance you push the joystick forwards and to the left, and the bot will face, forwards
         * and to the left, it wont keep rotating, it will simply match your desired heading.
         */
        FACE_ANGLE,
        ;
    }

    final List<CANSparkMax> mMotors;
    final WPI_PigeonIMU mPigeon;

    final MecanumDriveKinematics mKinematics;
    final MecanumDrivePoseEstimator mPoseEstimator;

    final ProfiledPIDController mThetaPID = kThetaPIDController;
    Rotation2d mDesiredHeading = new Rotation2d();
    TurningMode mTurningMode = kTurningModeDefault;

    boolean mFieldOriented = kFieldOrientedDefault;

    boolean mTargetLock = false;
    Rotation2d mTargetLockHeading = new Rotation2d();

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

        kThetaPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /** @param motor The motor to setup. */
    private void initMotor(CANSparkMax motor) {
        motor.setSmartCurrentLimit(kStallCurrentLimit, kFreeSpeedCurrentLimit);
        motor.setIdleMode(IdleMode.kCoast);
        motor.getPIDController().setP(kWheelPGain);
        motor.getEncoder().setPositionConversionFactor(kPositionConversionFactor);
        motor.getEncoder().setVelocityConversionFactor(kVelocityConversionFactor);
        if (Robot.isSimulation()) REVPhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNEO(1));
    }

    @Override
    public void periodic() {
        mPoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getHeading(), getWheelSpeeds());
    }

    @Override
    public void simulationPeriodic() {
        // Update motors.
        REVPhysicsSim.getInstance().run();
        // Update heading, (Not accurate, but functional).
        var deg =
                Units.radiansToDegrees(
                                mKinematics.toChassisSpeeds(getWheelSpeeds()).omegaRadiansPerSecond)
                        * Robot.kDefaultPeriod;
        mPigeon.getSimCollection().addHeading(deg);
    }

    /** Disables all motor output. */
    public void disable() {
        mMotors.forEach(CANSparkMax::disable);
    }

    /**
     * @param estimatedPose The estimated pose from vision target.
     * @param timestamp The timestamp the measurement is from.
     */
    public void addVisionMeasurement(Pose2d estimatedPose, double timestamp) {
        mPoseEstimator.addVisionMeasurement(estimatedPose, timestamp);
    }

    /**
     * @param x Input in the forwards direction. [-1, 1]
     * @param y Input in the left direction. [-1, 1]
     * @param z_x X axis of the desired heading AND turning input. [-1, 1]
     * @param z_y Y axis of the desired heading. [-1, 1]
     * @return A command which will drive the robot based off of the suppliers given.
     */
    public Command drive(
            DoubleSupplier x, DoubleSupplier y, DoubleSupplier z_x, DoubleSupplier z_y) {
        return new RunCommand(
                        () ->
                                drive(
                                        x.getAsDouble(),
                                        y.getAsDouble(),
                                        z_x.getAsDouble(),
                                        z_y.getAsDouble()),
                        this)
                .perpetually();
    }

    /**
     * @param x Input in the forwards direction. [-1, 1]
     * @param y Input in the left direction. [-1, 1]
     * @param z_x X axis of the desired heading AND turning input. [-1, 1]
     * @param z_y Y axis of the desired heading. [-1, 1]
     */
    public void drive(double x, double y, double z_x, double z_y) {

        // determine desired heading
        boolean noZInput = false;
        if (z_x == 0.0 && z_y == 0.0) {
            noZInput = true;
        }
        mDesiredHeading =
                switch (mTurningMode) {
                    case STANDARD -> {
                        // Allows for standard roatation concept, but allows it to be controlled
                        // by the PID controller, which will also keep it constrained.
                        if (mTargetLock && noZInput) yield mTargetLockHeading;
                        else
                            yield new Rotation2d(Math.atan2(z_x, -Math.abs(z_x) + 1))
                                    .rotateBy(getHeading());
                    }
                    case HOLD -> {
                        yield mDesiredHeading;
                    }
                    case FACE_ANGLE -> {
                        if (mTargetLock && noZInput) yield mTargetLockHeading;
                        if (noZInput) yield kDefaultHeading;
                        else yield new Rotation2d(Math.atan2(z_x, z_y));
                    }
                    default -> {
                        yield kDefaultHeading;
                    }
                };

        Vector2d input = new Vector2d(x * kMaxTranslationVelocity, y * kMaxTranslationVelocity);
        // Rotate inputs for field oriented driving.
        if (mFieldOriented) input.rotate(-getHeading().getDegrees());

        // Convert chassis speeds to individual wheel speeds.
        MecanumDriveWheelSpeeds wheelSpeeds =
                mKinematics.toWheelSpeeds(
                        new ChassisSpeeds(
                                input.x,
                                input.y,
                                mThetaPID.calculate(
                                        getHeading().getRadians(), mDesiredHeading.getRadians())));

        setWheelSpeeds(wheelSpeeds);
    }

    //
    // SETTERS
    //

    /** @param heading The heading to lock to. Set heading to null to cancel target lock. */
    private void setTargetLock(Rotation2d heading) {
        if (heading != null) {
            mTargetLockHeading = heading;
            mTargetLock = true;
        } else {
            mTargetLock = false;
        }
    }

    /** Enable target locking. */
    public void enableTargetLock(Rotation2d heading) {
        setTargetLock(heading);
    }

    /** Disable target locking. */
    public void disableTargetLock() {
        setTargetLock(null);
    }

    /** Sets the turning mode to {@link TurningMode#STANDARD}. */
    public void setStandardTurning() {
        setTurningMode(TurningMode.STANDARD);
    }

    /** Sets the turning mode to {@link TurningMode#HOLD}. */
    public void setHoldTurning() {
        setTurningMode(TurningMode.HOLD);
    }

    /** Sets the turning mode to {@link TurningMode#FACE_ANGLE}. */
    public void setFaceAngleTurning() {
        setTurningMode(TurningMode.FACE_ANGLE);
    }

    /** @param turningMode {@link TurningMode} to set the drivetrain to. */
    public void setTurningMode(TurningMode turningMode) {
        mTurningMode = turningMode;
    }

    /** @param fieldOriented Whether to drive field oriented or robot oriented. */
    public void setFieldOriented(boolean fieldOriented) {
        mFieldOriented = fieldOriented;
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
     * Tell the drivetrain where it is, this does not feed it an estimation, it blatently says, you
     * are here rn.
     *
     * @param pose The pose.
     * @param heading The heading.
     */
    public void setPose(Pose2d pose, Rotation2d heading) {
        mPoseEstimator.resetPosition(pose, heading);
        mMotors.forEach((motor) -> motor.getEncoder().setPosition(0));
    }

    /** @param targetSpeeds The wheel speeds to have the robot attempt to achieve. */
    public void setWheelSpeeds(MecanumDriveWheelSpeeds targetSpeeds) {
        // Ensure desired motor speeds are within acceptable values.
        targetSpeeds.desaturate(kMaxTranslationVelocity);

        // Convert wheel speeds to a list, because it makes applying it easier.
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

    /** Reset heading to 0. */
    public void resetGyro() {
        mPigeon.reset();
    }

    //
    // GETTERS
    //

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
        // return mPigeon.getRotation2d();
        double rads = mPigeon.getRotation2d().getRadians();

        while (rads > Math.PI) rads -= 2 * Math.PI;
        while (rads < -Math.PI) rads += 2 * Math.PI;
        return new Rotation2d(rads);
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
