/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.tigerbotics7125.robot.Robot;
import io.github.tigerbotics7125.robot.constants.RobotConstants;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger;

public class Intake extends SubsystemBase {
    CANSparkMax mMaster;
    CANSparkMax mSlave;

    AnalogInput mOccupancySensor;
    Trigger mOccupancy;

    DoubleSolenoid mSolenoid;

    ShuffleboardTab mSBTab;

    public Intake() {
        mMaster = new CANSparkMax(11, MotorType.kBrushless);
        mSlave = new CANSparkMax(12, MotorType.kBrushless);
        mSlave.follow(mMaster, true);

        mOccupancySensor = new AnalogInput(1);
        mOccupancy = new Trigger(() -> mOccupancySensor.getVoltage() < 2.0);

        mSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

        // create the acutal triggers
        mOccupancy = mOccupancy.debounce(.2);
        mOccupancy.trigger(this::clamp);
        mOccupancy.not().trigger(this::release);

        mSBTab = Shuffleboard.getTab("Intake");
        mSBTab.addBoolean("Occupancy", () -> mOccupancy.get());
        mSBTab.addDouble("sensorVolts", mOccupancySensor::getVoltage);
    }

    /**
     * Configures a motor.
     *
     * @param motor The motor to setup.
     */
    public void configureMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();

        motor.setIdleMode(IdleMode.kCoast);
        motor.setSmartCurrentLimit(8, 2, 550);
        motor.enableVoltageCompensation(RobotConstants.kNominalVoltage);

        motor.burnFlash();

        if (Robot.isSimulation())
            REVPhysicsSim.getInstance().addSparkMax(motor, 19.40175484f, 550.0f);
    }

    /** Disables all motor and pneumatic output. */
    public void disable() {
        mSolenoid.set(Value.kOff);
    }

    /** Clamp down the intake. */
    public void clamp() {
        mSolenoid.set(Value.kForward);
    }

    /** Release the spit the intake. */
    public void release() {
        mSolenoid.set(Value.kReverse);
    }

    /** Set the motors to intake. */
    public void intake() {
        mMaster.set(0.5);
    }

    /** Set the motors to barely pull inwards to grip. */
    public void intakeGrip() {
        mMaster.set(0.05);
    }

    /** Set the motors to eject */
    public void eject() {
        mMaster.set(-0.5);
    }
}
