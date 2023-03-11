/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.subsystem;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static io.github.tigerbotics7125.robot.constants.IntakeConstants.*;
import static io.github.tigerbotics7125.robot.constants.RobotConstants.PNEUMATICS_MODULE_TYPE;
import static io.github.tigerbotics7125.tigerlib.input.trigger.Trigger.ActivationCondition.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.tigerbotics7125.robot.Robot;
import io.github.tigerbotics7125.robot.constants.RobotConstants;

public class Intake extends SubsystemBase {
    CANSparkMax mMaster;
    CANSparkMax mSlave;

    SparkMaxPIDController mController;

    DoubleSolenoid mSolenoid;

    LinearFilter mCurrentFilter = LinearFilter.movingAverage(10);
    double mOutputCurrent = 0.0;

    ShuffleboardTab mSBTab;

    public Intake() {
        mMaster = new CANSparkMax(MASTER_ID, MotorType.kBrushless);
        mSlave = new CANSparkMax(SLAVE_ID, MotorType.kBrushless);
        mSlave.follow(mMaster, true);

        mController = mMaster.getPIDController();

        mSolenoid = new DoubleSolenoid(PNEUMATICS_MODULE_TYPE, FORWARDS_CHANNEL, REVERSE_CHANNEL);

        mSBTab = Shuffleboard.getTab("Intake");
        mSBTab.addBoolean("Claw", () -> mSolenoid.get().equals(Value.kForward));
        mSBTab.addDouble("Master Motor Current", () -> Robot.mPDH.getCurrent(5));
        mSBTab.addDouble("Slave Current", () -> Robot.mPDH.getCurrent(6));
    }

    /**
     * Configures a motor.
     *
     * @param motor The motor to setup.
     */
    public void configureMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();

        motor.setIdleMode(IDLE_MODE);
        motor.setSmartCurrentLimit(STALL_CURRENT_LIMIT_AMPS, FREE_SPEED_CURRENT_LIMIT_AMPS, 0);
        motor.enableVoltageCompensation(RobotConstants.NOMINAL_VOLTAGE);

        motor.burnFlash();

        if (Robot.isSimulation())
            REVPhysicsSim.getInstance()
                    .addSparkMax(motor, STALL_TORQUE_NEWTON_METERS, FREE_SPEED_RPM);
    }

    /** Disables all motor and pneumatic output. */
    public void disable() {
        mMaster.disable();
        mSlave.disable();
    }

    public double getCurrent() {
        return mOutputCurrent;
    }

    /**
     * @param grip Whether to grip or not grip the grippers.
     * @return A Command which will control the gripper pneumatics.
     */
    public CommandBase setGrippers(boolean grip) {
        return runOnce(() -> mSolenoid.set(grip ? GRIP_DIRECTION : RELEASE_DIRECTION));
    }

    /**
     * @param dutyCycle The duty cycle to run the motor at, [-1, 1].
     * @return A Command which will run the intake motors.
     */
    public CommandBase runIntake(double dutyCycle) {
        return run(() -> mController.setReference(dutyCycle, ControlType.kDutyCycle));
    }

    /** @return A Command which will open the gripper. */
    public CommandBase grippersOpen() {
        return setGrippers(false);
    }

    /** @return A Command which will lose the gripper. */
    public CommandBase grippersClose() {
        return setGrippers(true);
    }

    /** @return A Command which will run the intake inwards for collection. */
    public CommandBase intakeIn() {
        return runIntake(INTAKE_IN_SPEED);
    }

    /** @return A Command which will run the intake outwards for ejection. */
    public CommandBase intakeOut() {
        return runIntake(INTAKE_OUT_SPEED);
    }

    /** @return A Command which will run the intake inwards very slowly, to hold a gamepiece. */
    public CommandBase intakeHold() {
        return runIntake(INTAKE_HOLD_SPEED);
    }

    /** @return A Command which will close the grippers, then run the intake to hold a gamepiece. */
    public CommandBase holdGamepiece() {
        return grippersClose().andThen(runIntake(INTAKE_HOLD_SPEED));
    }

    /**
     * @return A Command which will run the intake, then will detect when a gamepiece is intaked,
     *     then close and hold it.
     */
    public CommandBase intakeAuto() {
        Debouncer debounce = new Debouncer(1, Debouncer.DebounceType.kRising);
        return runOnce(() -> debounce.calculate(false))
                .andThen(grippersOpen())
                .andThen(intakeIn())
                .until(() -> debounce.calculate(getCurrent() >= INTAKE_STALL_CURRENT))
                .finallyDo((interrupted) -> holdGamepiece().schedule());
    }

    /** Set the motors to pull inwards slowly. */
    public void grip() {
        mMaster.set(0.05);
    }

    /** Set the motors to eject */
    public void eject() {
        mMaster.set(-1);
    }

    @Override
    public void periodic() {
        mOutputCurrent = mCurrentFilter.calculate(mMaster.getOutputCurrent());
    }
}
