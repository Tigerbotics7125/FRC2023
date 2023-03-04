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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.tigerbotics7125.robot.PowerMonitor;
import io.github.tigerbotics7125.robot.Robot;
import io.github.tigerbotics7125.robot.constants.RobotConstants;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger;

public class Intake extends SubsystemBase {
    CANSparkMax mMaster;
    CANSparkMax mSlave;

    DoubleSolenoid mSolenoid;

    ShuffleboardTab mSBTab;

    public Intake() {
        mMaster = new CANSparkMax(MASTER_ID, MotorType.kBrushless);
        mSlave = new CANSparkMax(SLAVE_ID, MotorType.kBrushless);
        mSlave.follow(mMaster, true);

        mSolenoid = new DoubleSolenoid(PNEUMATICS_MODULE_TYPE, FORWARDS_CHANNEL, REVERSE_CHANNEL);

        double ampThreshold = 2;
        new Trigger(() -> PowerMonitor.getInstance().getCurrent(5) > ampThreshold)
                .trigger(this.grabObject());

        mSBTab = Shuffleboard.getTab("Intake");
        mSBTab.addBoolean("Claw", () -> mSolenoid.get().equals(Value.kForward));
        mSBTab.addDouble("Master Motor Current", () -> PowerMonitor.getInstance().getCurrent(5));
        mSBTab.addDouble("Slave Current", () -> PowerMonitor.getInstance().getCurrent(6));
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
        this.open();
        mMaster.disable();
        mSlave.disable();
    }

    /**
     * @return A Command which will close the intake, and run the wheels slowly as to keep the
     *     object from slipping out.
     */
    public Command grabObject() {
        var command =
                Commands.sequence(
                        Commands.run(this::intake).withTimeout(.15),
                        Commands.parallel(Commands.runOnce(this::close), Commands.run(this::grip)));
        command.addRequirements(this);
        return command;
    }

    /**
     * @return A Command which will open the intake, and run the wheels backwords as to get rid of a
     *     held object.
     */
    public Command releaseObject() {
        var command =
                Commands.sequence(
                        Commands.run(this::eject).withTimeout(.15), Commands.runOnce(this::open));
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
        mMaster.set(1);
    }

    /** Set the motors to pull inwards slowly. */
    public void grip() {
        mMaster.set(0.05);
    }

    /** Set the motors to eject */
    public void eject() {
        mMaster.set(-1);
    }
}
