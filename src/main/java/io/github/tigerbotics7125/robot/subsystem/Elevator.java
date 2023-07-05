/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.subsystem;

import static io.github.tigerbotics7125.robot.constants.ElevatorConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import io.github.tigerbotics7125.robot.Robot;
import io.github.tigerbotics7125.robot.constants.RobotConstants;

public class Elevator extends ProfiledPIDSubsystem {

    public enum State {
        HOME(0),
        QUARTER(MAX_HEIGHT / 4D),
        HALF(MAX_HEIGHT / 2D),
        THREE_QUARTERS(3D * MAX_HEIGHT / 4D),
        MAX(MAX_HEIGHT),

        GROUND_INTAKE(MAX_HEIGHT * .9), // .30),
        HYBRID(0),
        MID_CUBE(0),
        MID_CONE_PLACE(0),
        MID_CONE_SCORE(0),
        HIGH_CUBE_SCORE(MAX_HEIGHT),
        HIGH_CONE_PLACE(0),
        HIGH_CONE_SCORE(0);

        private double mMeters;

        private State(double meters) {
            mMeters = meters;
        }
    }

    private final CANSparkMax mMaster = new CANSparkMax(MASTER_ID, MOTOR_TYPE);
    private final CANSparkMax mFollower = new CANSparkMax(FOLLOWER_ID, MOTOR_TYPE);

    private final RelativeEncoder mEnc = mMaster.getEncoder();

    private final ElevatorFeedforward mFF = new ElevatorFeedforward(0.0, KG, KV, KA);

    private double mVelocity0 = 0D;
    private State mState = State.HOME;

    // Smooths out sim motion, not real representation of momentum.
    private LinearFilter mSimMomentum = LinearFilter.movingAverage(50);

    public Elevator() {
        super(PID);
        configMotor(mMaster);
        configMotor(mFollower);
        Timer.delay(.25);
        mMaster.setInverted(true);
        mFollower.follow(mMaster, true);
    }

    public void configMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();

        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(40);
        motor.enableVoltageCompensation(RobotConstants.NOMINAL_VOLTAGE);

        RelativeEncoder enc = motor.getEncoder();
        enc.setPositionConversionFactor(POS_CONV_FACTOR);
        enc.setVelocityConversionFactor(VEL_CONV_FACTOR);

        motor.burnFlash();
    }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        double ff =
                mFF.calculate(setpoint.velocity, (setpoint.velocity - mVelocity0) / Robot.kDefaultPeriod);
        mVelocity0 = setpoint.velocity;
        mMaster.setVoltage(output * 12D + ff);
    }

    @Override
    public double getMeasurement() {
        return mEnc.getPosition();
    }

    public State getState() {
        return mState;
    }

    public CommandBase setState(State state) {
        return runOnce(
                () -> {
                    if (!isEnabled()) this.enable();
                    mState = state;
                    setGoal(state.mMeters);
                });
    }

    @Override
    public void simulationPeriodic() {
        int dev = SimDeviceDataJNI.getSimDeviceHandle("SPARK MAX [" + mMaster.getDeviceId() + "]");
        SimDouble preVel =
                new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Velocity Pre-Conversion"));
        SimDouble vel = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Velocity"));

        double neoVel = RobotConstants.NEO_FREE_SPEED_RPM;
        double motorVel = MathUtil.clamp(mMaster.getAppliedOutput(), -1, 1) * neoVel;

        preVel.set(motorVel); // mSimMomentum.calculate(motorVel));
        vel.set(preVel.get() * VEL_CONV_FACTOR);
        double masterPos = vel.get() * .02;
        mEnc.setPosition(mEnc.getPosition() + masterPos);
    }
}
