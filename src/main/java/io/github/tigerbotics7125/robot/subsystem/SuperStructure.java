/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.subsystem;

import static io.github.tigerbotics7125.robot.constants.SuperStructureConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
        tab.addDouble(
                "forwardLimit", () -> mElevMaster.getSoftLimit(SoftLimitDirection.kForward));
        tab.addDouble(
                "reverseLimit", () -> mElevMaster.getSoftLimit(SoftLimitDirection.kReverse));

        if (Robot.isSimulation()) {
            tab.add("superstruc", mMech);
        }
    }

    public void configElevatorMotor(CANSparkMax motor) {
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

    public void configArmMotor(CANSparkMax motor) {

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

    private CommandBase runElevator(double value, ControlType controlType) {
        return run(() -> mElevMaster.getPIDController().setReference(value, controlType));
    }

    private CommandBase runArm(double value, ControlType controlType) {
        return run(() -> mArm.getPIDController().setReference(value, controlType));
    }

    private CommandBase runWrist(double value, ControlType controlType) {
        return run(() -> mArm.getPIDController().setReference(value, controlType));
    }

    public CommandBase disable() {
        return run( () -> {
            mElevMaster.disable();
        mArm.disable();
        mWrist.disable();
        });
    }

    public CommandBase elevatorUp() {
        return runElevator(.7, ControlType.kDutyCycle).withName("Elevator Up");
    }

    public CommandBase elevatorDown() {
        return runElevator(-.7, ControlType.kDutyCycle).withName("Elevator Down");
    }

    public CommandBase homeElev() {
        Debouncer fastDeb = new Debouncer(.075, DebounceType.kRising);
        Debouncer slowDeb = new Debouncer(.2, DebounceType.kRising);
        double fastHomeStallAmps = 5;
        double slowHomeStallAmps = 7;
        DoubleSupplier avgCurrent = () -> (mElevMaster.getOutputCurrent() + mElevFollower.getOutputCurrent()) / 2D;

        CommandBase clearDebouncer = runOnce(() -> {
            fastDeb.calculate(false);
            slowDeb.calculate(false);
        });
        BooleanSupplier fastStallDetect = () -> fastDeb.calculate(avgCurrent.getAsDouble() > fastHomeStallAmps);
        BooleanSupplier slowStallDetect = () -> slowDeb.calculate(avgCurrent.getAsDouble() > slowHomeStallAmps);

        CommandBase fastDown = runElevator(-.5, ControlType.kDutyCycle);
        CommandBase slowDown = runElevator(-.1, ControlType.kDutyCycle);
        CommandBase raiseSetDistance = runElevator(1, ControlType.kPosition);

        CommandBase setHome = runOnce(() -> {
            mElevMaster.getEncoder().setPosition(0);
            mElevFollower.getEncoder().setPosition(0);
        });

        CommandBase fastSequence = clearDebouncer
                .andThen(fastDown.until(fastStallDetect)).andThen(setHome);
        CommandBase slowSequence = clearDebouncer
                .andThen(slowDown.until(slowStallDetect)).andThen(setHome);

        return fastSequence.andThen(raiseSetDistance).andThen(slowSequence).andThen(runOnce(() -> mElevHasBeenHomed = true));
    }

    public CommandBase homeIfNeeded() {
        if (!mElevHasBeenHomed)
            return homeElev();
        else
            return Commands.none();
    }

    public CommandBase elevatorPosition(double position) {
        return homeIfNeeded().andThen(runElevator(position, ControlType.kPosition).until(() -> Math.abs(mElevMaster.getEncoder().getPosition() - position) < .5));
    }

    @Override
    public void periodic() {
        mElevatorMech.setLength(mElevMaster.getEncoder().getPosition());
        mArmMech.setAngle(Math.toDegrees(mArm.getEncoder().getPosition()));
        mWristMech.setAngle(Math.toDegrees(mWrist.getEncoder().getPosition()));
    }
}
