/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.subsystem;

import static io.github.tigerbotics7125.robot.constants.ArmConstants.*;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import io.github.tigerbotics7125.robot.constants.RobotConstants;

public class Arm extends ProfiledPIDSubsystem {

    public enum State {
        HOME(new Rotation2d()),
        GROUND_INTAKE(Rotation2d.fromDegrees(-10)),
        HIGH_CUBE(Rotation2d.fromDegrees(0)),
        UP(Rotation2d.fromRadians(.1));

        private Rotation2d mRotation;

        private State(Rotation2d rotation) {
            mRotation = rotation;
        }
    }

    private final CANSparkMax mArm = new CANSparkMax(MOTOR_ID, MOTOR_TYPE);
    private final CANCoder mCANCoder = new CANCoder(CANCODER_ID);

    private State mState = State.HOME;

    public Arm() {
        super(PID);

        configMotor(mArm);

        mCANCoder.configAllSettings(CANCODER_CONFIG);
        mCANCoder.setPosition(
                -(CANCODER_POS_CONV_FACTOR
                        * (mCANCoder.getAbsolutePosition() - ABSOLUTE_HOME_DEG)));
    }

    public void configMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();

        motor.setIdleMode(IdleMode.kCoast);
        motor.setInverted(true);
        motor.setSmartCurrentLimit(40);
        motor.enableVoltageCompensation(RobotConstants.NOMINAL_VOLTAGE);

        RelativeEncoder enc = motor.getEncoder();
        enc.setPositionConversionFactor(POS_CONV_FACTOR);
        enc.setVelocityConversionFactor(VEL_CONV_FACTOR);
    }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        mArm.setVoltage(output * 12D + FEEDFORWARD.calculate(setpoint.position, setpoint.velocity));
    }

    @Override
    public double getMeasurement() {
        return Units.degreesToRadians(mCANCoder.getPosition()) * CANCODER_POS_CONV_FACTOR;
    }

    public State getState() {
        return mState;
    }

    public CommandBase setState(State state) {
        return runOnce(
                () -> {
                    this.enable();
                    mState = state;
                    setGoal(state.mRotation.getRadians());
                });
    }

    @Override
    public void simulationPeriodic() {
        int dev = SimDeviceDataJNI.getSimDeviceHandle("SPARK MAX [" + mArm.getDeviceId() + "]");
        SimDouble preVel =
                new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Velocity Pre-Conversion"));
        SimDouble vel = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Velocity"));

        double neoVel = RobotConstants.NEO_FREE_SPEED_RPM;
        double motorVel = MathUtil.clamp(mArm.getAppliedOutput(), -1, 1) * neoVel;

        preVel.set(motorVel);
        vel.set(preVel.get() * VEL_CONV_FACTOR);
        double masterPos = vel.get() * .02;
        mArm.getEncoder().setPosition(mArm.getEncoder().getPosition() + masterPos);
    }
}
