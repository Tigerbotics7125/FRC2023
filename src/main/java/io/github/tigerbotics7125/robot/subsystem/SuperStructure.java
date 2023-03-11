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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.tigerbotics7125.lib.Analog_873M_UltraSonic;
import io.github.tigerbotics7125.lib.Analog_873M_UltraSonic.SensorType;
import io.github.tigerbotics7125.robot.Robot;
import io.github.tigerbotics7125.robot.constants.RobotConstants;

public class SuperStructure extends SubsystemBase {

    private enum BrakeState {
        BRAKING,
        COASTING
    }

    private BrakeState mArmBrakeState = BrakeState.BRAKING;

    private Analog_873M_UltraSonic mElevatorDistanceSensor =
            new Analog_873M_UltraSonic(SensorType.ANALOG_VOLTAGE, ELEVATOR_HEIGHT_SENSOR_RAW);
    private CANSparkMax mElevatorMaster = new CANSparkMax(ELEVATOR_MASTER_ID, MOTOR_TYPE);
    private CANSparkMax mElevatorSlave = new CANSparkMax(ELEVATOR_SLAVE_ID, MOTOR_TYPE);

    private CANSparkMax mArm = new CANSparkMax(ARM_MOTOR_ID, MOTOR_TYPE);

    private CANSparkMax mWrist = new CANSparkMax(WRIST_MOTOR_ID, MOTOR_TYPE);

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
        configElevatorMotor(mElevatorMaster);
        configElevatorMotor(mElevatorSlave);
        mElevatorMaster.setInverted(true);
        mElevatorSlave.follow(mElevatorMaster, true);

        configArmMotor(mArm);
        configWristMotor(mWrist);

        var tab = Shuffleboard.getTab("Super Structure");
        tab.addDouble("Elevator%", mElevatorDistanceSensor::getPercentage);
        tab.addDouble(
                "forwardLimit", () -> mElevatorMaster.getSoftLimit(SoftLimitDirection.kForward));
        tab.addDouble(
                "reverseLimit", () -> mElevatorMaster.getSoftLimit(SoftLimitDirection.kReverse));

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
    }

    private void configWristMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(20, 5, 0);
        motor.enableVoltageCompensation(RobotConstants.NOMINAL_VOLTAGE);

        SparkMaxPIDController pid = motor.getPIDController();
        pid.setP(1);
        pid.setI(0);
        pid.setD(0);

        motor.burnFlash();

        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNEO(1));
        }
    }

    public void disable() {
        mElevatorMaster.disable();
        mArm.disable();
        mWrist.disable();
    }

    public Command elevatorUp() {
        return Commands.run(() -> elevatorDuty(.7));
    }

    public Command elevatorDown() {
        return Commands.run(() -> elevatorDuty(-.7));
    }

    public void elevatorDuty(double elevatorDuty) {
        mElevatorMaster.getPIDController().setReference(elevatorDuty, ControlType.kDutyCycle);
    }

    public void armDuty(double armDuty) {
        mArm.getPIDController().setReference(armDuty, ControlType.kDutyCycle);
    }

    public void wristDuty(double wristDuty) {
        mWrist.getPIDController().setReference(wristDuty, ControlType.kDutyCycle);
    }

    @Override
    public void periodic() {
        mElevatorMech.setLength(mElevatorMaster.getEncoder().getPosition());
        mArmMech.setAngle(Math.toDegrees(mArm.getEncoder().getPosition()));
        mWristMech.setAngle(Math.toDegrees(mWrist.getEncoder().getPosition()));
    }
}
