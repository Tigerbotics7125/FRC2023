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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator2 extends SubsystemBase {

    public enum ElevState {
        DISABLED(0),

        HOME(0),

        GROUND_INTAKE(.3),
        MID_CUBE(.7),
        HIGH_CUBE(1.1),

        QUARTER(MAX_HEIGHT / 4D),
        HALF(MAX_HEIGHT / 2D),
        THREE_QUARTERS(3D * MAX_HEIGHT / 4D),
        MAX(MAX_HEIGHT);

        private double mMeters;

        private ElevState(double meters) {
            mMeters = meters;
        }
    }

    private final CANSparkMax mMaster = new CANSparkMax(MASTER_ID, MOTOR_TYPE);
    private final CANSparkMax mFollower = new CANSparkMax(FOLLOWER_ID, MOTOR_TYPE);

    private final RelativeEncoder mEnc = mMaster.getEncoder();

    private ElevState mCurrentState = ElevState.DISABLED;
    private PIDController mPID = new PIDController(P_GAIN, I_GAIN, D_GAIN);

    public Elevator2() {
        configMotor(mMaster);
        configMotor(mFollower);
        Timer.delay(.2);
        mMaster.setInverted(true);
        mFollower.follow(mMaster, true);
    }

    private void configMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        Timer.delay(.2);

        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(40);

        var enc = motor.getEncoder();
        enc.setPositionConversionFactor(POS_CONV_FACTOR);
        enc.setVelocityConversionFactor(VEL_CONV_FACTOR);

        motor.burnFlash();
        Timer.delay(.2);
    }

    public CommandBase setState(ElevState state) {
        return Commands.either(
                Commands.none(),
                runOnce(
                        () -> {
                            mCurrentState = state;
                        }),
                () -> mCurrentState == state);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elev Measure", getMeasurement());
        if (mCurrentState == ElevState.DISABLED) {
            mMaster.set(0);
            return;
        }

        mMaster.set(PID.calculate(getMeasurement(), mCurrentState.mMeters));
    }

    public double getMeasurement() {
        return mEnc.getPosition();
    }
}
