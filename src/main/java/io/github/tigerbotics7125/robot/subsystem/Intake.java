/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.subsystem;

import static io.github.tigerbotics7125.robot.constants.IntakeConstants.*;
import static io.github.tigerbotics7125.robot.constants.RobotConstants.PNEUMATICS_MODULE_TYPE;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.tigerbotics7125.robot.Robot;
import io.github.tigerbotics7125.robot.constants.RobotConstants;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger;

public class Intake extends SubsystemBase {
    CANSparkMax mMaster;
    CANSparkMax mSlave;

    AnalogInput mOccupancySensor1;
    AnalogInput mOccupancySensor2;
    Trigger mOccupancy1;
    Trigger mOccupancy2;

    DoubleSolenoid mSolenoid;

    ShuffleboardTab mSBTab;

    public Intake() {
        mMaster = new CANSparkMax(MASTER_ID, MotorType.kBrushless);
        mSlave = new CANSparkMax(SLAVE_ID, MotorType.kBrushless);
        mSlave.follow(mMaster, true);

        mOccupancySensor1 = new AnalogInput(OCCUPANCY_1_PORT);
        mOccupancySensor2 = new AnalogInput(OCCUPANCY_2_PORT);

        mSolenoid = new DoubleSolenoid(PNEUMATICS_MODULE_TYPE, FORWARDS_CHANNEL, REVERSE_CHANNEL);

        // create the acutal triggers
        mOccupancy1 =
                new Trigger(() -> mOccupancySensor1.getVoltage() <= OCCUPANCY_VOLTAGE_THRESHOLD)
                        .debounce(OCCUPANCY_DEBOUNCE_TIME);
        mOccupancy2 =
                new Trigger(() -> mOccupancySensor2.getVoltage() <= OCCUPANCY_VOLTAGE_THRESHOLD)
                        .debounce(OCCUPANCY_DEBOUNCE_TIME);

        Trigger joinedOccupancy = mOccupancy1.and(mOccupancy2);
        joinedOccupancy.trigger(grabObject());

        mSBTab = Shuffleboard.getTab("Intake");
        mSBTab.addBoolean("Occupancy", joinedOccupancy::get);
        mSBTab.addDouble("Sensor 1 Volts", mOccupancySensor1::getVoltage);
        mSBTab.addDouble("Sensor 2 Volts", mOccupancySensor2::getVoltage);
        mSBTab.addBoolean("Claw", () -> mSolenoid.get().equals(Value.kForward));
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

        motor.setOpenLoopRampRate(OPEN_LOOP_RAMP_RATE);

        motor.burnFlash();

        if (Robot.isSimulation())
            REVPhysicsSim.getInstance()
                    .addSparkMax(motor, STALL_TORQUE_NEWTON_METERS, FREE_SPEED_RPM);
    }

    /** Disables all motor and pneumatic output. */
    public void disable() {
        mSolenoid.set(Value.kOff);
    }

    /**
     * @return A Command which will close the intake, and run the wheels slowly as to keep the
     *     object from slipping out.
     */
    public Command grabObject() {
        ParallelCommandGroup command =
                new ParallelCommandGroup(Commands.runOnce(this::close), Commands.run(this::grip));
        command.addRequirements(this);
        return command;
    }

    /**
     * @return A Command which will open the intake, and run the wheels backwords as to get rid of a
     *     held object.
     */
    public Command releaseObject() {
        ParallelCommandGroup command =
                new ParallelCommandGroup(
                        Commands.runOnce(this::open), Commands.run(this::eject).withTimeout(.5));
        command.addRequirements(this);
        return command;
    }

    /** Close the intake. */
    public void close() {
        mSolenoid.set(Value.kForward);
    }

    /** Open the intake. */
    public void open() {
        mSolenoid.set(Value.kReverse);
    }

    /** Set the motors to intake. */
    public void intake() {
        mMaster.set(0.5);
    }

    /** Set the motors to pull inwards slowly. */
    public void grip() {
        mMaster.set(0.05);
    }

    /** Set the motors to eject */
    public void eject() {
        mMaster.set(-0.5);
    }
}
