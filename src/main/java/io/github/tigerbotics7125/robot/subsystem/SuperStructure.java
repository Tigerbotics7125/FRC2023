/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.subsystem;

import static io.github.tigerbotics7125.robot.constants.SuperStructureConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.tigerbotics7125.lib.Analog_873M_UltraSonic;
import io.github.tigerbotics7125.lib.Analog_873M_UltraSonic.SensorType;
import io.github.tigerbotics7125.robot.Robot;
import io.github.tigerbotics7125.robot.constants.RobotConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * This class controls the "super" structure of the robot. It features 3DOF, a linear elevator,
 * jointed to an arm, jointed again to a wrist.
 */
public class SuperStructure extends SubsystemBase {

    private Analog_873M_UltraSonic mElevatorDistanceSensor =
            new Analog_873M_UltraSonic(SensorType.ANALOG_VOLTAGE, ELEVATOR_HEIGHT_SENSOR_RAW);
    private CANSparkMax mElevMaster = new CANSparkMax(ELEV_MASTER_ID, MOTOR_TYPE);
    private CANSparkMax mElevFollower = new CANSparkMax(ELEV_FOLLOWER_ID, MOTOR_TYPE);

    private boolean mElevHasBeenHomed = false;

    private CANSparkMax mArm = new CANSparkMax(ARM_ID, MOTOR_TYPE);

    private CANSparkMax mWrist = new CANSparkMax(WRIST_ID, MOTOR_TYPE);

    private Mechanism2d mMech = new Mechanism2d(2, 2);
    private MechanismRoot2d mRoot = mMech.getRoot("robot", 0.394837, 0.226789);
    private MechanismLigament2d mElevatorMech =
            mRoot.append(
                    new MechanismLigament2d(
                            "Elevator",
                            ELEV_START_DIST,
                            ELEV_ANGLE.getDegrees(),
                            5,
                            new Color8Bit(Color.kRed)));
    private MechanismLigament2d mArmMech =
            mElevatorMech.append(
                    new MechanismLigament2d(
                            "Arm",
                            ARM_LENGTH,
                            ARM_START_ANGLE.getDegrees(),
                            5,
                            new Color8Bit(Color.kBlue)));
    private MechanismLigament2d mWristMech =
            mArmMech.append(
                    new MechanismLigament2d(
                            "Wrist",
                            WRIST_LENGTH,
                            WRIST_START_ANGLE.getDegrees(),
                            5,
                            new Color8Bit(Color.kOrange)));

    public SuperStructure() {
        configElevatorMotor(mElevMaster);
        configElevatorMotor(mElevFollower);
        mElevMaster.setInverted(true);
        mElevFollower.follow(mElevMaster, true);

        configArmMotor(mArm);
        configWristMotor(mWrist);

        var tab = Shuffleboard.getTab("Super Structure");
        tab.addDouble("Elevator%", mElevatorDistanceSensor::getPercentage);
        tab.addDouble("forwardLimit", () -> mElevMaster.getSoftLimit(SoftLimitDirection.kForward));
        tab.addDouble("reverseLimit", () -> mElevMaster.getSoftLimit(SoftLimitDirection.kReverse));

        if (Robot.isSimulation()) {
            tab.add("superstruc", mMech);
        }
    }

    private void configElevatorMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();

        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(40);
        motor.enableVoltageCompensation(RobotConstants.NOMINAL_VOLTAGE);

        SparkMaxPIDController pid = motor.getPIDController();
        pid.setP(1);
        pid.setI(0);
        pid.setD(0);
        pid.setOutputRange(-1, 1);

        motor.burnFlash();

        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNEO(1));
        }
        Timer.delay(.25);
    }

    private void configArmMotor(CANSparkMax motor) {

        motor.restoreFactoryDefaults();

        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(20, 5, 0);
        motor.enableVoltageCompensation(RobotConstants.NOMINAL_VOLTAGE);

        SparkMaxPIDController pid = motor.getPIDController();
        pid.setP(1);
        pid.setI(0);
        pid.setD(0);
        pid.setOutputRange(-1, 1);

        motor.burnFlash();

        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNEO(1));
        }
        Timer.delay(.25);
    }

    private void configWristMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(20, 5, 0);
        motor.enableVoltageCompensation(RobotConstants.NOMINAL_VOLTAGE);
        motor.setInverted(true);

        SparkMaxPIDController pid = motor.getPIDController();
        pid.setP(1);
        pid.setI(0);
        pid.setD(0);

        motor.burnFlash();

        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNEO(1));
        }
        Timer.delay(.25);
    }

    // ! PRIVATE INTERNAL COMMANDS

    /**
     * @param value What value to control the elevator with.
     * @param controlType How to control the elevator.
     * @return A Command which will run the elevator as desired.
     */
    private CommandBase runElevator(double value, ControlType controlType) {
        return run(() -> mElevMaster.getPIDController().setReference(value, controlType));
    }

    /**
     * @param value What value to control the arm with.
     * @param controlType How to control the arm.
     * @return A Command which will run the arm as desired.
     */
    // TODO: needs to be private
    public CommandBase runArm(double value, ControlType controlType) {
        return run(() -> mArm.getPIDController().setReference(value, controlType));
    }

    /**
     * @param value What value to control the wrist with.
     * @param controlType How to control the wrist.
     * @return A Command which will run the wrist as desired.
     */
    // TODO: needs to be private
    public CommandBase runWrist(double value, ControlType controlType) {
        return run(() -> mArm.getPIDController().setReference(value, controlType));
    }

    /**
     * @param idleMode What state the motor should use at idle.
     * @return A Command which sets the new idle state of all motors in the structure.
     */
    private CommandBase idleAll(IdleMode idleMode) {
        return runOnce(
                () -> {
                    mElevMaster.setIdleMode(idleMode);
                    mElevFollower.setIdleMode(idleMode);
                    mArm.setIdleMode(idleMode);
                    mWrist.setIdleMode(idleMode);
                });
    }

    // ! PUBLIC EXTERNAL COMMANDS

    /** @return A Command which disables all motor output. */
    public CommandBase disable() {
        return run(
                () -> {
                    mElevMaster.disable();
                    mArm.disable();
                    mWrist.disable();
                });
    }

    /** @return A Command which sets all motors to coast. */
    public CommandBase coastAll() {
        return idleAll(IdleMode.kCoast);
    }

    /** @return A Command which sets all motors to brake. */
    public CommandBase brakeAll() {
        return idleAll(IdleMode.kBrake);
    }

    /** @return A Command which moves the elevator up, using duty cycle (ie its boring). */
    public CommandBase elevatorUp() {
        return runElevator(.7, ControlType.kDutyCycle).withName("Elevator Up");
    }

    /** @return A Command which moves the elevator down, using duty cycle (ie its boring). */
    public CommandBase elevatorDown() {
        return runElevator(-.7, ControlType.kDutyCycle).withName("Elevator Down");
    }

    /**
     * @return A Command which will home the elevator at the bottom, and calibrate its known
     *     position.
     */
    public CommandBase homeingSequence() {
        Debouncer fastDeb = new Debouncer(.075, DebounceType.kRising);
        Debouncer slowDeb = new Debouncer(.2, DebounceType.kRising);
        double fastHomeStallAmps = 5;
        double slowHomeStallAmps = 7;
        DoubleSupplier avgCurrent =
                () -> (mElevMaster.getOutputCurrent() + mElevFollower.getOutputCurrent()) / 2D;

        CommandBase clearDebouncer =
                runOnce(
                        () -> {
                            fastDeb.calculate(false);
                            slowDeb.calculate(false);
                        });
        BooleanSupplier fastStallDetect =
                () -> fastDeb.calculate(avgCurrent.getAsDouble() > fastHomeStallAmps);
        BooleanSupplier slowStallDetect =
                () -> slowDeb.calculate(avgCurrent.getAsDouble() > slowHomeStallAmps);

        CommandBase fastDown = runElevator(-.5, ControlType.kDutyCycle);
        CommandBase slowDown = runElevator(-.1, ControlType.kDutyCycle);
        CommandBase raiseSetDistance = runElevator(1, ControlType.kPosition);

        CommandBase setHome =
                runOnce(
                        () -> {
                            mElevMaster.getEncoder().setPosition(0);
                            mElevFollower.getEncoder().setPosition(0);
                        });

        CommandBase fastSequence =
                clearDebouncer.andThen(fastDown.until(fastStallDetect)).andThen(setHome);
        CommandBase slowSequence =
                clearDebouncer.andThen(slowDown.until(slowStallDetect)).andThen(setHome);

        return fastSequence
                .andThen(raiseSetDistance)
                .andThen(slowSequence)
                .andThen(runOnce(() -> mElevHasBeenHomed = true));
    }

    /**
     * @return A Command which will run the homing sequence if it has not been run since the code
     *     has started.
     */
    public CommandBase homeIfNeeded() {
        if (!mElevHasBeenHomed) return homeingSequence();
        else return Commands.none();
    }

    /**
     * This command will run the homing sequence if it has not run yet.
     *
     * @param position Position to attempt to achieve.
     * @return A Command which will attempt to achieve the provided position on the elevator.
     */
    public CommandBase elevatorPosition(double position) {
        BooleanSupplier atPosition =
                () -> {
                    double currPos = mElevMaster.getEncoder().getPosition();
                    double delta = Math.abs(currPos - position);
                    return delta < 0.5;
                };

        return homeIfNeeded()
                .andThen(runElevator(position, ControlType.kPosition).until(atPosition));
    }

    @Override
    public void periodic() {
        mElevatorMech.setLength(mElevMaster.getEncoder().getPosition());
        mArmMech.setAngle(Math.toDegrees(mArm.getEncoder().getPosition()));
        mWristMech.setAngle(Math.toDegrees(mWrist.getEncoder().getPosition()));
    }
}
