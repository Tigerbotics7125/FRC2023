/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.subsystem;

import static io.github.tigerbotics7125.robot.constants.SuperStructureConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.tigerbotics7125.lib.MB1122;
import io.github.tigerbotics7125.robot.Robot;
import io.github.tigerbotics7125.robot.constants.RobotConstants;

public class SuperStructure extends SubsystemBase {

    private enum BrakeState {
        BRAKING,
        COASTING
    }

    private BrakeState mArmBrakeState = BrakeState.BRAKING;

    private MB1122 mElevatorDistanceSensor = new MB1122(ELEVATOR_HEIGHT_SENSOR_RAW);
    private CANSparkMax mElevatorMaster = new CANSparkMax(ELEVATOR_MASTER_ID, MOTOR_TYPE);
    private CANSparkMax mElevatorSlave = new CANSparkMax(ELEVATOR_SLAVE_ID, MOTOR_TYPE);

    private CANSparkMax mArm = new CANSparkMax(ARM_MOTOR_ID, MOTOR_TYPE);

    private CANSparkMax mWrist = new CANSparkMax(WRIST_MOTOR_ID, MOTOR_TYPE);

    public SuperStructure() {
        configElevatorMotor(mElevatorMaster);
        configElevatorMotor(mElevatorSlave);
        mElevatorSlave.follow(mElevatorMaster, true);
        configArmMotor(mArm);
        configWristMotor(mWrist);
    }

    public void configElevatorMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();

        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(40);
        motor.enableVoltageCompensation(RobotConstants.NOMINAL_VOLTAGE);

        // TODO: Find ideal soft limit locations and encoder positions at that value.
        // Should be able to auto configure OTF with the distance sensor.
        /**
         * motor.enableSoftLimit(SoftLimitDirection.kForward, true);
         * motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
         */
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
        return Commands.runOnce(() -> elevatorDuty(.7));
    }

    public Command elevatorDown() {
        return Commands.runOnce(() -> elevatorDuty(-.7));
    }

    public void elevatorDuty(double elevatorDuty) {
        mElevatorMaster.set(elevatorDuty);
    }

    public void armDuty(double armDuty) {
        mArm.set(armDuty);
    }

    public void wristDuty(double wristDuty) {
        mWrist.set(wristDuty);
    }
}
