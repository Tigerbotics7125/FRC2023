/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.subsystem;

import static io.github.tigerbotics7125.robot.constants.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.tigerbotics7125.robot.Robot;
import io.github.tigerbotics7125.robot.constants.RobotConstants;

public class Intake extends SubsystemBase {
    private final CANSparkMax mMaster;
    private final CANSparkMax mFollower;

    private final SparkMaxPIDController mController;

    private final DoubleSolenoid mSolenoid;

    private final LinearFilter mCurrentFilter;

    private final ShuffleboardTab mSBTab;

    private double mFilteredCurrent = 0.0;

    public Intake() {
        mMaster = new CANSparkMax(MASTER_ID, MotorType.kBrushless);
        mFollower = new CANSparkMax(FOLLOWER_ID, MotorType.kBrushless);
        configureMotor(mMaster);
        configureMotor(mFollower);

        mFollower.follow(mMaster, true);

        mController = mMaster.getPIDController();

        mSolenoid = Robot.mPH.makeDoubleSolenoid(FORWARDS_CHANNEL, REVERSE_CHANNEL);

        mCurrentFilter = LinearFilter.movingAverage(CURRENT_AVG_SAMPLES);

        mSBTab = Shuffleboard.getTab("Intake");
        mSBTab.addBoolean("Grippers", () -> getGrippers());
        mSBTab.addDouble("Filtered Current", () -> mFilteredCurrent);
        mSBTab.addString(
                "Current Command",
                () -> {
                    Command cmd = this.getCurrentCommand();
                    return cmd == null ? "No Commands Running" : cmd.getName();
                });
    }

    /**
     * Configures a motor.
     *
     * @param motor The motor to setup.
     */
    public void configureMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();

        motor.setIdleMode(IDLE_MODE);
        motor.setSmartCurrentLimit(STALL_CURRENT_LIMIT_AMPS);
        motor.enableVoltageCompensation(RobotConstants.NOMINAL_VOLTAGE);

        motor.burnFlash();

        if (Robot.isSimulation())
            REVPhysicsSim.getInstance()
                    .addSparkMax(motor, STALL_TORQUE_NEWTON_METERS, FREE_SPEED_RPM);
        Timer.delay(.25);
    }

    /** @return The average and filtered current of the motors. */
    public double getCurrent() {
        return mFilteredCurrent;
    }

    /** @return Whether or not the grippers are engaged. */
    public boolean getGrippers() {
        return mSolenoid.get() == GRIP_DIRECTION;
    }

    /** @return A Command which will disable all motor output. */
    public CommandBase disable() {
        return run(() -> {
                    mMaster.disable();
                    mFollower.disable();
                })
                .withName("Disabled");
    }

    // PRIVATE raw setter commands.

    /**
     * @param grip Whether to grip or not grip the grippers.
     * @return A Command which will control the gripper pneumatics.
     */
    private CommandBase setGrippers(boolean grip) {
        return runOnce(() -> mSolenoid.set(grip ? GRIP_DIRECTION : RELEASE_DIRECTION));
    }

    /**
     * @param dutyCycle The duty cycle to run the motor at, [-1, 1].
     * @return A Command which will run the intake motors.
     */
    private CommandBase runIntake(double dutyCycle) {
        return run(() -> mController.setReference(dutyCycle, ControlType.kDutyCycle));
    }

    // PUBLIC commands

    /** @return A Command which will open the gripper. */
    public CommandBase grippersOpen() {
        return setGrippers(false).withName("Opening Grippers");
    }

    /** @return A Command which will lose the gripper. */
    public CommandBase grippersClose() {
        return setGrippers(true).withName("Closing Gripper");
    }

    /** @return A Command which will run the intake inwards for collection. */
    public CommandBase intakeIn() {
        return runIntake(INTAKE_IN_SPEED).withName("Intaking");
    }

    /** @return A Command which will run the intake outwards for ejection. */
    public CommandBase intakeOut() {
        return runIntake(INTAKE_OUT_SPEED).withName("Outaking");
    }

    /** @return A Command which will run the intake inwards very slowly, to hold a gamepiece. */
    public CommandBase intakeHold() {
        return runIntake(INTAKE_HOLD_SPEED).withName("Holdtaking");
    }

    /** @return A Command which will close the grippers, then run the intake to hold a gamepiece. */
    public CommandBase holdRoutine() {
        return grippersClose().andThen(runIntake(INTAKE_HOLD_SPEED)).withName("Hold Routine");
    }

    /**
     * @return A Command which will run the intake, then will detect when a gamepiece is intaked,
     *     then close and hold it.
     */
    public CommandBase intakeRoutine() {
        Debouncer debounce = new Debouncer(MIN_DETECT_TIME, Debouncer.DebounceType.kRising);
        return runOnce(() -> debounce.calculate(false))
                .andThen(grippersOpen())
                .andThen(intakeIn())
                .until(() -> debounce.calculate(getCurrent() >= MIN_DETECT_CURRENT))
                .finallyDo((interrupted) -> holdRoutine().schedule())
                .withName("Intake Routine");
    }

    @Override
    public void periodic() {
        // Update current average.
        double rawMaster = mMaster.getOutputCurrent();
        double rawFollower = mFollower.getOutputCurrent();
        double rawAverage = (rawMaster + rawFollower) / 2D;

        mFilteredCurrent = mCurrentFilter.calculate(rawAverage);
    }
}
