/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.subsystem;

import static io.github.tigerbotics7125.robot.constants.DrivetrainConstants.CAN.*;
import static io.github.tigerbotics7125.robot.constants.DrivetrainConstants.Characteristics.*;
import static io.github.tigerbotics7125.robot.constants.DrivetrainConstants.DefaultDrivingOptions.*;
import static io.github.tigerbotics7125.robot.constants.DrivetrainConstants.Kinematics.*;
import static io.github.tigerbotics7125.robot.constants.DrivetrainConstants.MotorValues.*;
import static io.github.tigerbotics7125.robot.constants.DrivetrainConstants.Odometry.*;
import static io.github.tigerbotics7125.robot.constants.RobotConstants.kNominalVoltage;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.tigerbotics7125.robot.Robot;
import io.github.tigerbotics7125.robot.constants.DrivetrainConstants;
import java.util.ArrayList;
import java.util.List;

public class Drivetrain extends SubsystemBase {
    public enum TurningMode {
        /** Joystick inputs are directly mapped to the rotation of the drivetrain. */
        JOYSTICK_DIRECT,

        /** The drivetrain rotates to face the same direction as the joystick. */
        JOYSTICK_ANGLE,

        /** The drivetrain maintains a fixed heading. */
        HEADING_LOCK,

        /** The drivetrain maintains a fixed orientation towards a target. */
        TARGET_LOCK;
    }

    private final List<CANSparkMax> mMotors;
    private final List<RelativeEncoder> mEncoders;
    private final List<SparkMaxPIDController> mPIDControllers;
    private final WPI_PigeonIMU mPigeon;

    private final MecanumDriveKinematics mKinematics;
    private final MecanumDrivePoseEstimator mPoseEstimator;

    private final ProfiledPIDController mThetaPID = kThetaPIDController;
    private Rotation2d mDesiredHeading = new Rotation2d();
    private TurningMode mTurningMode = kTurningMode;

    private boolean mFieldOriented = kFieldOriented;

    private boolean mTargetLock = false;
    private Rotation2d mTargetLockHeading = new Rotation2d();

    public Drivetrain() {
        mMotors =
                List.of(
                        new CANSparkMax(kFLID, kMotorType),
                        new CANSparkMax(kFRID, kMotorType),
                        new CANSparkMax(kRLID, kMotorType),
                        new CANSparkMax(kRRID, kMotorType));
        mEncoders = new ArrayList<>();
        mPIDControllers = new ArrayList<>();
        mMotors.forEach(this::initMotor);

        mPigeon = new WPI_PigeonIMU(new WPI_TalonSRX(kPigeonID));

        mKinematics = new MecanumDriveKinematics(kFLOffset, kFROffset, kRLOffset, kRROffset);
        mPoseEstimator =
                new MecanumDrivePoseEstimator(
                        mKinematics,
                        getHeading(),
                        getWheelPositions(),
                        new Pose2d(),
                        kStateStdDevs,
                        kVisionMeasurementStdDevs);

        kThetaPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /** @param motor The motor to setup. */
    private void initMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();

        motor.setIdleMode(IdleMode.kCoast);
        motor.setSmartCurrentLimit(kStallCurrentLimit, kFreeSpeedCurrentLimit);
        motor.enableVoltageCompensation(kNominalVoltage);

        RelativeEncoder encoder = motor.getEncoder();
        mEncoders.add(encoder);
        encoder.setPositionConversionFactor(kPositionConversionFactor);
        encoder.setVelocityConversionFactor(kVelocityConversionFactor);

        SparkMaxPIDController pid = motor.getPIDController();
        mPIDControllers.add(pid);
        pid.setP(kWheelPGain);
        pid.setI(kWheelIGain);
        pid.setD(kWheelDGain);
        pid.setOutputRange(-1, 1); // duty cycle

        // invert right side
        if (motor.getDeviceId() % 2 == 0) {
            motor.setInverted(true);
        } else {
            motor.setInverted(false);
        }

        // save config to smax
        motor.burnFlash();

        if (Robot.isSimulation())
            REVPhysicsSim.getInstance().addSparkMax(motor, kStallTorque, kFreeSpeed);
    }

    @Override
    public void periodic() {
        mPoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getHeading(), getWheelPositions());
    }

    @Override
    public void simulationPeriodic() {
        // Update motors.
        REVPhysicsSim.getInstance().run();
        // Update heading, (Not accurate, but functional).
        var deg =
                Units.radiansToDegrees(
                                mKinematics.toChassisSpeeds(getWheelSpeeds()).omegaRadiansPerSecond)
                        * .02; // 20ms loop time
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
     */
    public void drive(double x, double y, double z_x, double z_y) {

        // determine desired heading
        boolean noZInput = false;
        if (z_x == 0.0 && z_y == 0.0) {
            noZInput = true;
        }
        mDesiredHeading =
                switch (mTurningMode) {
                    case JOYSTICK_DIRECT -> {
                        // Allows for standard roatation concept, but allows it to be
                        // controlled
                        // by the PID controller, which will also keep it constrained.
                        if (mTargetLock && noZInput) yield mTargetLockHeading;
                        else
                            yield new Rotation2d(Math.atan2(z_x, -Math.abs(z_x) + 1))
                                    .rotateBy(getHeading());
                    }
                    case HEADING_LOCK -> {
                        yield mDesiredHeading;
                    }
                    case JOYSTICK_ANGLE -> {
                        if (mTargetLock && noZInput) yield mTargetLockHeading;
                        if (noZInput) yield kDefaultHeading;
                        else yield new Rotation2d(Math.atan2(z_x, z_y));
                    }
                    case TARGET_LOCK -> {
                        yield mTargetLockHeading;
                    }
                    default -> {
                        yield kDefaultHeading;
                    }
                };

        // Rotate inputs for field oriented driving.
        Translation2d input = new Translation2d(x * kMaxLinearVelocity, y * kMaxLinearVelocity);
        if (mFieldOriented) input = input.rotateBy(getHeading().unaryMinus());

        SmartDashboard.putNumber("desiredHeading", mDesiredHeading.getDegrees());
        SmartDashboard.putNumber("currentHeading", getHeading().getDegrees());

        // Convert chassis speeds to individual wheel speeds.
        double xSpeed = input.getX();
        double ySpeed = input.getY();
        double zSpeed =
                mThetaPID.calculate(getHeading().getRadians(), mDesiredHeading.getRadians()); // *
        // DrivetrainConstants.Characteristics.kMaxRotationalVelocity;
        ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zSpeed);
        setChassisSpeeds(desiredChassisSpeeds);
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(
                        () -> {
                            // Reset odometry for the first path you run during auto
                            if (isFirstPath) {
                                this.setPose(
                                        traj.getInitialHolonomicPose(),
                                        traj.getInitialHolonomicPose().getRotation());
                            }
                        }),
                new PPMecanumControllerCommand(
                        traj,
                        this::getPose,
                        mKinematics,
                        new PIDController(1, 0, 0),
                        new PIDController(1, 0, 0),
                        new PIDController(1, 0, 0),
                        kMaxLinearVelocity,
                        this::setWheelSpeeds,
                        this));
    }

    //
    // SETTERS
    //

    /** @param heading The heading to lock to. Set heading to null to cancel target lock. */
    public void setTargetLock(Rotation2d heading) {
        mTargetLockHeading = heading;
    }

    /** Enable target locking. */
    public void enableTargetLock(Rotation2d heading) {
        setTargetLock(heading);
    }

    /** Disable target locking. */
    public void disableTargetLock() {
        setTargetLock(null);
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
        mPoseEstimator.resetPosition(heading, getWheelPositions(), pose);
        mEncoders.forEach((encoder) -> encoder.setPosition(0));
    }

    public void setChassisSpeeds(ChassisSpeeds targetSpeeds) {
        setWheelSpeeds(mKinematics.toWheelSpeeds(targetSpeeds));
    }

    /** @param targetSpeeds The wheel speeds to have the robot attempt to achieve. */
    public void setWheelSpeeds(MecanumDriveWheelSpeeds targetSpeeds) {
        // Ensure desired motor speeds are within acceptable values.
        targetSpeeds.desaturate(DrivetrainConstants.Characteristics.kMaxLinearVelocity);

        double fl = targetSpeeds.frontLeftMetersPerSecond;
        double fr = targetSpeeds.frontRightMetersPerSecond;
        double rl = targetSpeeds.rearLeftMetersPerSecond;
        double rr = targetSpeeds.rearRightMetersPerSecond;

        List<Double> wheelSpeeds = List.of(fl, fr, rl, rr);

        for (int i = 0; i < 4; i++) {
            mPIDControllers.get(i).setReference(wheelSpeeds.get(i), ControlType.kVelocity);
            System.out.println();
            System.out.println("targetSpeed: " + wheelSpeeds.get(i));
            System.out.println(
                    "kylers number: "
                            + ((mEncoders.get(i).getVelocity() * 0.5) - wheelSpeeds.get(i))
                                    * DrivetrainConstants.Characteristics.kWheelPGain);
        }

        /*


        SmartDashboard.putString("desiredSpeeds", targetSpeeds.toString());

        double maxSpeed = DrivetrainConstants.Characteristics.kRunningLinearVelocity;
        double fl = targetSpeeds.frontLeftMetersPerSecond / maxSpeed;
        double fr = targetSpeeds.frontRightMetersPerSecond / maxSpeed;
        double rl = targetSpeeds.rearLeftMetersPerSecond / maxSpeed;
        double rr = targetSpeeds.rearRightMetersPerSecond / maxSpeed;

        // Convert wheel speeds to a list, because it makes applying it easier.
        List<Double> wheelSpeeds = List.of(fl, fr, rl, rr);

        SmartDashboard.putString("rpm setpoint", wheelSpeeds.toString());

        for (int i = 0; i < 4; i++) {
            double duty = wheelSpeeds.get(i);
            if (duty > 1.0) duty = 1.0;
            else if (duty < -1.0) duty = -1.0;
            mMotors.get(i).set(duty);
        }
        */
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
        return mPigeon.getRotation2d();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return mKinematics.toChassisSpeeds(getWheelSpeeds());
    }

    /** @return The current wheel speeds. */
    public MecanumDriveWheelSpeeds getWheelSpeeds() {

        double fl = mEncoders.get(0).getVelocity();
        double fr = mEncoders.get(1).getVelocity();
        double rl = mEncoders.get(2).getVelocity();
        double rr = mEncoders.get(3).getVelocity();

        var speeds = new MecanumDriveWheelSpeeds(fl, fr, rl, rr);
        SmartDashboard.putString("actualWheelSpeeds", speeds.toString());

        return speeds;
    }

    public MecanumDriveWheelPositions getWheelPositions() {

        double fl = mEncoders.get(0).getPosition();
        double fr = mEncoders.get(1).getPosition();
        double rl = mEncoders.get(2).getPosition();
        double rr = mEncoders.get(3).getPosition();

        return new MecanumDriveWheelPositions(fl, fr, rl, rr);
    }

    public TurningMode getTurningMode() {
        return mTurningMode;
    }

    public MecanumDriveKinematics getKinematics() {
        return mKinematics;
    }
}
