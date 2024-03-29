/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.subsystem;

import static io.github.tigerbotics7125.robot.constants.DrivetrainConstants.*;
import static io.github.tigerbotics7125.robot.constants.RobotConstants.NOMINAL_VOLTAGE;
import static io.github.tigerbotics7125.tigerlib.input.trigger.Trigger.ActivationCondition.ON_RISING;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.tigerbotics7125.lib.AllianceFlipUtil;
import io.github.tigerbotics7125.robot.Robot;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
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
    private final AHRS mNavx;

    // odometry
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

    public final Field2d mField = new Field2d();

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

        mNavx = new AHRS(SPI.Port.kMXP);

        mPoseEstimator =
                new MecanumDrivePoseEstimator(
                        KINEMATICS, getHeading(), getWheelPositions(), new Pose2d());

        new Trigger(RobotState::isDisabled)
                .trigger(
                        ON_RISING,
                        Commands.waitSeconds(5)
                                .andThen(() -> mMotors.forEach(m -> m.setIdleMode(IdleMode.kCoast)))
                                .ignoringDisable(true));

        new Trigger(RobotState::isEnabled)
                .trigger(
                        ON_RISING,
                        Commands.runOnce(
                                () -> mMotors.forEach(m -> m.setIdleMode(IdleMode.kBrake))));

        // Setup dashboard values.
        ShuffleboardTab driveTab = Shuffleboard.getTab("drive");
        driveTab.addNumber("Current Heading", () -> getHeading().getDegrees());
        driveTab.addNumber("Desired Heading", () -> mDesiredHeading.getDegrees());
        driveTab.addBoolean("Field Oriented", () -> mFieldOriented);
        driveTab.addString("Turning Mode", () -> mTurningMode.name());
        driveTab.add("Odometry", mField);
    }

    /**
     * Configures a motor, its encoder, and its pid controller.
     *
     * @param motor The motor to setup.
     */
    private void configureMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();

        motor.setIdleMode(IdleMode.kCoast);
        motor.setSmartCurrentLimit(STALL_CURRENT_LIMIT_AMPS);
        motor.enableVoltageCompensation(NOMINAL_VOLTAGE);

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

        mField.setRobotPose(getPose());
    }

    @Override
    public void simulationPeriodic() {
        // update sim because its wack sometimes.
        var deg =
                Units.radiansToDegrees(
                                KINEMATICS.toChassisSpeeds(getWheelSpeeds()).omegaRadiansPerSecond)
                        * .02; // 20ms loop time

        // https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java
        // I guess its better than no sim support...
        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        angle.set(mNavx.getFusedHeading() + deg);

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

    // ! FUNCTIONAL METHODS

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
                            .rotateBy(new Rotation2d(z_y * MAX_ANGULAR_VELOCITY / .02));
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
        if (mFieldOriented) input = input.rotateBy(getHeading());

        /** Rotation input, in units of %on for open loop, and rad/s for closed loop. */
        double thetaSpeed;
        if (openLoop) thetaSpeed = z_y;
        else
            thetaSpeed =
                    mThetaPID.calculate(getHeading().getRadians(), mDesiredHeading.getRadians());

        // set outputs
        if (openLoop) {
            setOpenLoopSpeeds(
                    MecanumDrive.driveCartesianIK(input.getX(), input.getY(), thetaSpeed));

        } else {
            setChassisSpeeds(new ChassisSpeeds(input.getX(), input.getY(), thetaSpeed));
        }
    }

    public void addVisionEstimate(EstimatedRobotPose estPose) {
        Pose2d pose = AllianceFlipUtil.apply(estPose.estimatedPose.toPose2d());
        mPoseEstimator.addVisionMeasurement(pose, estPose.timestampSeconds);
    }

    // ! GETTERS

    public TurningMode getTurningMode() {
        return mTurningMode;
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
        return mNavx.getRotation2d();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return KINEMATICS.toChassisSpeeds(getWheelSpeeds());
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

    public List<CANSparkMax> getLeftMotors() {
        return List.of(mMotors.get(0), mMotors.get(2));
    }

    public List<CANSparkMax> getRightMotors() {
        return List.of(mMotors.get(1), mMotors.get(3));
    }

    public double getGyroRate() {
        return -mNavx.getRate();
    }

    // ! SETTERS

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
     * @param pose The pose.
     * @param heading The heading.
     */
    public void resetOdometry(Pose2d pose) {
        mPoseEstimator.resetPosition(getHeading(), getWheelPositions(), pose);
    }

    /** Reset heading to 0. */
    public void resetGyro() {
        mNavx.reset();
    }

    /** @param targetSpeeds The chassis speed to try to achieve. */
    public void setChassisSpeeds(ChassisSpeeds targetSpeeds) {
        setClosedLoopSpeeds(KINEMATICS.toWheelSpeeds(targetSpeeds));
    }

    /**
     * Used for open loop driving.
     *
     * @param speeds The wheel speeds in duty cycle to have the robot drive by.
     */
    public void setOpenLoopSpeeds(MecanumDrive.WheelSpeeds speeds) {

        List<Double> duties =
                List.of(speeds.frontLeft, speeds.frontRight, speeds.rearLeft, speeds.rearRight);

        for (int i = 0; i < 4; i++) {
            mMotors.get(i).set(duties.get(i));
            // mPIDControllers.get(i).setReference(duties.get(i), ControlType.kDutyCycle);
        }
    }

    /** @param targetSpeeds The wheel speeds to have the robot attempt to achieve. */
    public void setClosedLoopSpeeds(MecanumDriveWheelSpeeds targetSpeeds) {
        // Ensure desired motor speeds are within acceptable values.
        targetSpeeds.desaturate(MAX_LINEAR_VELOCITY_MPS);

        List<Double> targetWheelSpeeds =
                List.of(
                        targetSpeeds.frontLeftMetersPerSecond,
                        targetSpeeds.frontRightMetersPerSecond,
                        targetSpeeds.rearLeftMetersPerSecond,
                        targetSpeeds.rearRightMetersPerSecond);

        for (int i = 0; i < 4; i++) {
            mPIDControllers
                    .get(i)
                    .setReference(
                            targetWheelSpeeds.get(i),
                            ControlType.kVelocity,
                            0,
                            mFF.calculate(targetWheelSpeeds.get(i)),
                            ArbFFUnits.kVoltage);
        }
    }

    // ! PUBLIC EXTERNAL COMMANDS

    /** @return A Command which disables all motor output. */
    public CommandBase disable() {
        return runOnce(() -> mMotors.forEach(CANSparkMax::disable));
    }

    /**
     * NOTE: This method requires openloop control.
     *
     * @param x X axis input, forwards is positive.
     * @param y Y axis input, left is positive.
     * @param z_x Rotation x input, forwards is positive. Used only for {@link
     *     TurningMode#JOYSTICK_ANGLE}.
     * @param z_y Rotation y input, left is positive. Also used for {@link
     *     TurningMode#JOYSTICK_DIRECT}, where positive correlates to CCW rotation.
     * @param limit Limiting supplier, all inputs are divided by this value, should be [0, 1]. 0
     *     meaning no output, 1 meaning full output
     * @return A Command which will drive the robot, limiting its speed according to the limit
     *     function.
     */
    public CommandBase driveWithLimits(
            final DoubleSupplier x,
            final DoubleSupplier y,
            final DoubleSupplier z_x,
            final DoubleSupplier z_y,
            final DoubleSupplier limit) {
        return drive(
                () -> x.getAsDouble() * limit.getAsDouble(),
                () -> y.getAsDouble() * limit.getAsDouble(),
                () -> z_x.getAsDouble() * limit.getAsDouble(),
                () -> z_y.getAsDouble() * limit.getAsDouble(),
                () -> true);
    }

    /**
     * @param x X axis input, forwards is positive.
     * @param y Y axis input, left is positive.
     * @param z_x Rotation x input, forwards is positive. Used only for {@link
     *     TurningMode#JOYSTICK_ANGLE}.
     * @param z_y Rotation y input, left is positive. Also used for {@link
     *     TurningMode#JOYSTICK_DIRECT}, where positive correlates to CCW rotation.
     * @param openLoop Whether to use open or closed loop control.
     * @return A Command which will drive the robot.
     * @see {@link Drivetrain#drive(double, double, double, double, boolean)}.
     */
    public CommandBase drive(
            DoubleSupplier x,
            DoubleSupplier y,
            DoubleSupplier z_x,
            DoubleSupplier z_y,
            BooleanSupplier openLoop) {
        return run(
                () ->
                        drive(
                                x.getAsDouble(),
                                y.getAsDouble(),
                                z_x.getAsDouble(),
                                z_y.getAsDouble(),
                                openLoop.getAsBoolean()));
    }

    /** @return A Command which changes the turning mode to {@link TurningMode#JOYSTICK_DIRECT}. */
    public CommandBase directTurning() {
        return Commands.runOnce(() -> setTurningMode(TurningMode.JOYSTICK_DIRECT));
    }

    /** @return A Command which changes the turning mode to {@link TurningMode#JOYSTICK_ANGLE}. */
    public CommandBase angleTurning() {
        return Commands.runOnce(() -> setTurningMode(TurningMode.JOYSTICK_ANGLE));
    }

    /** @return A Command which changes the turning mode to {@link TurningMode#HEADING_LOCK}. */
    public CommandBase lockTurning() {
        return Commands.runOnce(() -> setTurningMode(TurningMode.HEADING_LOCK));
    }
}
