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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm2 extends SubsystemBase {

    public enum ArmState {
        DISABLED(null),

        STOW(Rotation2d.fromDegrees(10)),
        GROUND_INTAKE(Rotation2d.fromDegrees(50)),
        MID_CUBE(Rotation2d.fromDegrees(15)),
        HIGH_CUBE(Rotation2d.fromDegrees(15)),

        TEST(Rotation2d.fromDegrees(45));

        public final Rotation2d mAngle;

        ArmState(Rotation2d angle) {
            mAngle = angle;
        }
    }

    private final CANSparkMax mArm = new CANSparkMax(MOTOR_ID, MOTOR_TYPE);
    private final RelativeEncoder mRelEnc = mArm.getEncoder();
    private final CANCoder mAbsEnc = new CANCoder(CANCODER_ID);

    private ArmState mCurrentState = ArmState.DISABLED;

    public Arm2() {
        configMotor(mArm);
        // mAbsEnc.configAllSettings(CANCODER_CONFIG);
        // set the encoder (refering to the arm pos considering the conversion factor) to the
        // absolute encoder pos at the arm.
        mRelEnc.setPosition(mAbsEnc.getAbsolutePosition() * CHAIN_RATIO / 360);
        // PID.setGoal(90);
    }

    public void configMotor(CANSparkMax smax) {
        smax.restoreFactoryDefaults();
        Timer.delay(.2);

        smax.setIdleMode(IdleMode.kBrake);
        smax.setInverted(true);
        smax.setSmartCurrentLimit(40);

        var enc = smax.getEncoder();
        enc.setPositionConversionFactor(POS_CONV_FACTOR);
        enc.setVelocityConversionFactor(VEL_CONV_FACTOR);

        smax.burnFlash();
        Timer.delay(.2);
    }

    public CommandBase setState(ArmState state) {
        return Commands.either(
                Commands.none(),
                runOnce(
                        () -> {
                            mCurrentState = state;
                            // PID.(mCurrentState.mAngle.getDegrees());
                        }),
                () -> mCurrentState == state);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Measurement", getAngle());

        if (mCurrentState == ArmState.DISABLED) {
            mArm.set(0);
            return;
        }

        mArm.set(PID.calculate(getAngle(), mCurrentState.mAngle.getDegrees()));

        // mArm.set(PID.calculate(getAngle()));
    }

    /** @return Degrees? */
    public double getAngle() {
        return mRelEnc.getPosition() / POS_CONV_FACTOR;
    }
}
