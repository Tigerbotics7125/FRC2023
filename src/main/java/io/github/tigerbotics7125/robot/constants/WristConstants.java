/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.constants;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class WristConstants {

    public static final MotorType MOTOR_TYPE = MotorType.kBrushless;

    public static final int MOTOR_ID = 31;
    public static final int CANCODER_ID = 31;

    // Theoretical values, neo free speed (in rps) * pos_conv_factor
    public static final double MAX_VEL = .5; // 1.76; // rotations per second
    // not calculated, simply pick time to reach max vel.
    public static final double MAX_ACCEL = MAX_VEL / 1D; // rotations per second^2

    public static final double P_GAIN = .4;
    public static final double I_GAIN = 0;
    public static final double D_GAIN = 0;
    public static final Constraints CONSTRAINTS = new Constraints(MAX_VEL, MAX_ACCEL);
    public static final ProfiledPIDController PID =
            new ProfiledPIDController(P_GAIN, I_GAIN, D_GAIN, CONSTRAINTS);

    static {
        PID.setTolerance(Units.degreesToRadians(1));
    }

    public static final double ABSOLUTE_HOME_DEG = 117D;

    public static final double GEAR_RATIO = 20D;
    public static final double CHAIN_RATIO = 60D / 12D;
    public static final double CHAIN_RATIO2 = 12D / 22D;
    public static final double CANCODER_POS_CONV_FACTOR = 1D / CHAIN_RATIO2;
    public static final double POS_CONV_FACTOR = 1D / (GEAR_RATIO * CHAIN_RATIO * CHAIN_RATIO2);
    public static final double VEL_CONV_FACTOR = POS_CONV_FACTOR / 60D; // mps

    public static final CANCoderConfiguration CANCODER_CONFIG = new CANCoderConfiguration();

    static {
        CANCODER_CONFIG.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        CANCODER_CONFIG.initializationStrategy =
                SensorInitializationStrategy.BootToAbsolutePosition;
        CANCODER_CONFIG.magnetOffsetDegrees = 0D; // TODO: find this value
        CANCODER_CONFIG.unitString = "deg";
    }
}
