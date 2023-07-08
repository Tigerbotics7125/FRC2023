/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.subsystem;

import static io.github.tigerbotics7125.robot.constants.WristConstants.*;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist2 extends SubsystemBase {

    public enum WristState {
        DISABLED(null),

        STOW(Rotation2d.fromDegrees(8)),

        GROUND_INTAKE(Rotation2d.fromDegrees(25)),
        MID_CUBE(Rotation2d.fromDegrees(15)),
        HIGH_CUBE(Rotation2d.fromDegrees(15));

        public Rotation2d mAngle;

        WristState(Rotation2d angle) {
            mAngle = angle;
        }
    }

    private final CANSparkMax mWrist = new CANSparkMax(MOTOR_ID, MOTOR_TYPE);
    private final RelativeEncoder mRelEnc = mWrist.getEncoder();
    private final CANCoder mAbsEnc = new CANCoder(CANCODER_ID);

    private WristState mCurrentState = WristState.DISABLED;

    public Wrist2() {
        configMotor(mWrist);

        mRelEnc.setPosition(CHAIN_RATIO2);
    }

    private void configMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        Timer.delay(.2);

        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(false);
        motor.setSmartCurrentLimit(30);

        var enc = motor.getEncoder();
        enc.setPositionConversionFactor(POS_CONV_FACTOR);
        enc.setVelocityConversionFactor(VEL_CONV_FACTOR);

        motor.burnFlash();
        Timer.delay(.2);
    }

    public CommandBase setState(WristState state) {
        return Commands.either(
                Commands.none(),
                runOnce(() -> mCurrentState = state),
                () -> mCurrentState == state);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Measurement", getAngle());

        if (mCurrentState == WristState.DISABLED) {
            mWrist.set(0);
            return;
        }

        mWrist.set(PID.calculate(getAngle(), mCurrentState.mAngle.getDegrees()));
    }

    /** @return magik */
    public double getAngle() {
        return mRelEnc.getPosition() / POS_CONV_FACTOR;
    }
}
