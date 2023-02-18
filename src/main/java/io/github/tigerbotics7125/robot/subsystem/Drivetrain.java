/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.subsystem;

import static io.github.tigerbotics7125.robot.constants.DrivetrainConstants.*;
import static io.github.tigerbotics7125.robot.constants.RobotConstants.kNominalVoltage;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.tigerbotics7125.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.photonvision.EstimatedRobotPose;

/**
 * This class controls the mecanum drivetrain of the robot. It features open and closed loop control
 * of the wheels, feedforward, PID controlled heading, odometry and vision pose estimation.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
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

    // controllers and sensors
    private final List<CANSparkMax> mMotors;
    private final List<RelativeEncoder> mEncoders;
    private final List<SparkMaxPIDController> mPIDControllers;
    private final WPI_PigeonIMU mPigeon;

    // odometry
    private final MecanumDriveKinematics mKinematics;
    private final MecanumDrivePoseEstimator mPoseEstimator;

    // pid control
    private final ProfiledPIDController mThetaPID = THETA_PID_CONTROLLER;
    private Rotation2d mDesiredHeading = new Rotation2d();

    // driving options
    private boolean mFieldOriented = FIELD_ORIENTED_DEFAULT;
    private TurningMode mTurningMode = TURNING_MODE_DEFAULT;

    // feed forward control
    // TODO: add ff constants to DT constants file.
    private SimpleMotorFeedforward mFF = new SimpleMotorFeedforward(0.1, 2.8, 1.5);

    /**
     * @param cameras A list of all camera-translation pairs used on the robot for vision detection,
     *     this is for calculating robot pose from vision.
     */
    public Drivetrain() {
        mMotors =
                List.of(
                        new CANSparkMax(FRONT_LEFT_ID, MOTOR_TYPE),
                        new CANSparkMax(FRONT_RIGHT_ID, MOTOR_TYPE),
                        new CANSparkMax(REAR_LEFT_ID, MOTOR_TYPE),
                        new CANSparkMax(REAR_RIGHT_ID, MOTOR_TYPE));
        mEncoders = new ArrayList<>();
        mPIDControllers = new ArrayList<>();
        mMotors.forEach(this::configureMotor);

        mPigeon = new WPI_PigeonIMU(new WPI_TalonSRX(PIGEON_ID));

        mKinematics = KINEMATICS;
        mPoseEstimator =
                new MecanumDrivePoseEstimator(
                        mKinematics, getHeading(), getWheelPositions(), new Pose2d());

        // Setup dashboard values.
        ShuffleboardTab driveTab = Shuffleboard.getTab("drive");
        driveTab.addNumber("Current Heading (rad)", () -> getHeading().getRadians());
        driveTab.addNumber("Desired Heading (rad)", () -> mDesiredHeading.getRadians());
        driveTab.addBoolean("Field Oriented (bool)", () -> mFieldOriented);
        driveTab.addString("Turning Mode (str)", () -> mTurningMode.name());
    }

    /**
     * Configures a motor, its encoder, and its pid controller.
     *
     * @param motor The motor to setup.
     */
    private void configureMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();

        motor.setIdleMode(IdleMode.kCoast);
        motor.setSmartCurrentLimit(
                STALL_CURRENT_LIMIT_AMPS, FREE_SPEED_CURRENT_LIMIT_AMPS, (int) FREE_SPEED_RPM);
        motor.enableVoltageCompensation(kNominalVoltage);

        RelativeEncoder encoder = motor.getEncoder();
        mEncoders.add(encoder);
        encoder.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
        encoder.setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);

        SparkMaxPIDController pid = motor.getPIDController();
        mPIDControllers.add(pid);
        pid.setP(WHEEL_P_GAIN);
        pid.setI(WHEEL_I_GAIN);
        pid.setD(WHEEL_D_GAIN);
        pid.setOutputRange(-1, 1); // duty cycle

        // invert right side
        if (motor.getDeviceId() % 2 == 0) {
            motor.setInverted(true);
        } else {
            motor.setInverted(false);
        }

        // save config to motor controller
        motor.burnFlash();

        if (Robot.isSimulation())
            REVPhysicsSim.getInstance()
                    .addSparkMax(motor, STALL_TORQUE_NEWTON_METERS, FREE_SPEED_RPM);
    }

    @Override
    public void periodic() {
        // update the pose estimation with the current odometry data.
        mPoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getHeading(), getWheelPositions());
    }

    @Override
    public void simulationPeriodic() {
        // update sim because its wack sometimes.
        var deg =
                Units.radiansToDegrees(
                                mKinematics.toChassisSpeeds(getWheelSpeeds()).omegaRadiansPerSecond)
                        * .02; // 20ms loop time
        mPigeon.getSimCollection().addHeading(deg);

        /**
         * The following wheel position shenenigan fixes the super slow movement in sim. I still
         * don't know whats going on, the velocity is correct but the position is stupid so here is
         * the easy solution; just set it manually based on velocity duh.
         */
        var ws = getWheelSpeeds();
        // convert speed to meters given the loop time.
        double flm = ws.frontLeftMetersPerSecond * .02;
        double frm = ws.frontRightMetersPerSecond * .02;
        double rlm = ws.rearLeftMetersPerSecond * .02;
        double rrm = ws.rearRightMetersPerSecond * .02;

        List<Double> pos = List.of(flm, frm, rlm, rrm);

        for (int i = 0; i < 4; i++) {
            mEncoders.get(i).setPosition(mEncoders.get(i).getPosition() + pos.get(i));
        }
    }

    /** Disables all motor output. */
    public void disable() {
        mMotors.forEach(CANSparkMax::disable);
    }

    public void addVisionEstimate(EstimatedRobotPose pose) {
        mPoseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
    }

    /**
     * The rotation axes are the same orientation as the translational ones, pretend you are looking
     * straight down from above the robot, left-right is the y axis, and front-back is the x axis.
     *
     * @param x X axis input, forwards is positive.
     * @param y Y axis input, left is positive.
     * @param z_x Rotation x input, forwards is positive. Used only for {@link
     *     TurningMode#JOYSTICK_ANGLE}.
     * @param z_y Rotation y input, left is positive. Also used for {@link
     *     TurningMode#JOYSTICK_DIRECT}, where positive correlates to CCW rotation.
     * @param openLoop Whether to use open or closed loop control.
     * @return A Command which uses the given suppliers and drives the drivetrain.
     */
    public Command driveCmd(
            DoubleSupplier x,
            DoubleSupplier y,
            DoubleSupplier z_x,
            DoubleSupplier z_y,
            boolean openLoop) {
        return Commands.run(
                () ->
                        this.drive(
                                x.getAsDouble(),
                                y.getAsDouble(),
                                z_x.getAsDouble(),
                                z_y.getAsDouble(),
                                openLoop),
                this);
    }

    /**
     * The rotation axes are the same orientation as the translational ones, pretend you are looking
     * straight down from above the robot, left-right is the y axis, and front-back is the x axis.
     *
     * @param x X axis input, forwards is positive.
     * @param y Y axis input, left is positive.
     * @param z_x Rotation x input, forwards is positive. Used only for {@link
     *     TurningMode#JOYSTICK_ANGLE}.
     * @param z_y Rotation y input, left is positive. Also used for {@link
     *     TurningMode#JOYSTICK_DIRECT}, where positive correlates to CCW rotation.
     * @param openLoop Whether to use open or closed loop control.
     */
    public void drive(double x, double y, double z_x, double z_y, boolean openLoop) {

        // Determines if there is rotation input.
        boolean noRotationInput = z_x == 0.0 && z_y == 0.0;
        // Determine a desired heading depending on the turning mode.
        mDesiredHeading =
                switch (mTurningMode) {
                    case JOYSTICK_DIRECT -> getHeading()
                            .rotateBy(new Rotation2d(z_y, -Math.abs(z_y) + 1));
                    case JOYSTICK_ANGLE -> noRotationInput
                            ? HEADING_DEFAULT
                            : new Rotation2d(z_x, z_y);
                    case HEADING_LOCK -> mDesiredHeading;
                    default -> HEADING_DEFAULT;
                };

        /**
         * Input translation, in units of %on for open loop, and m/s for closed loop, if this
         * drivetrain is in field oriented control; then the input will be rotated to accomodate.
         */
        Translation2d input = new Translation2d(x, y);
        if (!openLoop) input = input.times(MAX_LINEAR_VELOCITY_MPS);
        if (mFieldOriented) input = input.rotateBy(getHeading().unaryMinus());

        /** Rotation input, in units of %on for open loop, and rad/s for closed loop. */
        double thetaSpeed;
        if (openLoop) {
            thetaSpeed = z_y;
        } else {
            thetaSpeed =
                    mThetaPID.calculate(getHeading().getRadians(), mDesiredHeading.getRadians());
        }

        // set outputs
        if (openLoop) {
            setWheelSpeeds(MecanumDrive.driveCartesianIK(input.getX(), input.getY(), thetaSpeed));
        } else {
            setChassisSpeeds(new ChassisSpeeds(input.getX(), input.getY(), thetaSpeed));
        }
    }

    //
    // SETTERS
    //

    /** @param turningMode {@link TurningMode} to set the drivetrain to. */
    public void setTurningMode(TurningMode turningMode) {
        mTurningMode = turningMode;
    }

    /** @param fieldOriented Whether to drive field oriented or robot oriented. */
    public void setFieldOriented(boolean fieldOriented) {
        mFieldOriented = fieldOriented;
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

    /**
     * Used for open loop driving.
     *
     * @param speeds The wheel speeds in %on to have the robot drive by.
     */
    public void setWheelSpeeds(MecanumDrive.WheelSpeeds speeds) {
        List<Double> duties =
                List.of(speeds.frontLeft, speeds.frontRight, speeds.rearLeft, speeds.rearRight);

        for (int i = 0; i < 4; i++) {
            mMotors.get(i).set(duties.get(i));
        }
    }

    /** @param targetSpeeds The wheel speeds to have the robot attempt to achieve. */
    public void setWheelSpeeds(MecanumDriveWheelSpeeds targetSpeeds) {
        // Ensure desired motor speeds are within acceptable values.
        targetSpeeds.desaturate(MAX_LINEAR_VELOCITY_MPS);

        double flTarg = targetSpeeds.frontLeftMetersPerSecond;
        double frTarg = targetSpeeds.frontRightMetersPerSecond;
        double rlTarg = targetSpeeds.rearLeftMetersPerSecond;
        double rrTarg = targetSpeeds.rearRightMetersPerSecond;

        List<Double> targets = List.of(flTarg, frTarg, rlTarg, rrTarg);

        for (int i = 0; i < 4; i++) {
            mPIDControllers
                    .get(i)
                    .setReference(
                            targets.get(i),
                            ControlType.kVelocity,
                            0,
                            mFF.calculate(targets.get(i)),
                            ArbFFUnits.kVoltage);
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

        return new MecanumDriveWheelSpeeds(fl, fr, rl, rr);
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

    public List<CANSparkMax> getLeftMotors() {
        return List.of(mMotors.get(0), mMotors.get(2));
    }

    public List<CANSparkMax> getRightMotors() {
        return List.of(mMotors.get(1), mMotors.get(3));
    }

    public double getGyroRate() {
        return -mPigeon.getRate();
    }
}
