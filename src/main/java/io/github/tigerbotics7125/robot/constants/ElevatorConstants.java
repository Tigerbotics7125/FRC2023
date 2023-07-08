/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.constants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class ElevatorConstants {

    public static final MotorType MOTOR_TYPE = MotorType.kBrushless;

    public static final int MASTER_ID = 11;
    public static final int FOLLOWER_ID = 12;

    public static final double KS = 0.058628;
    public static final double KV = 13.43400;
    public static final double KA = 1.289400;
    public static final double KG = 0.032181;
    public static final ElevatorFeedforward mFF = new ElevatorFeedforward(KS, KG, KV, KA);

    // Theoretical values, neo free speed * pos_conv_factor
    public static final double MAX_VEL = 0.84; // mps
    // Emperical value.
    public static final double MAX_ACCEL = MAX_VEL / 1.125; // mps^s

    public static final double P_GAIN = 20;
    public static final double I_GAIN = 0;
    public static final double D_GAIN = .1;
    public static final Constraints CONSTRAINTS = new Constraints(MAX_VEL, MAX_ACCEL);
    public static final double TOLERANCE = .05; // 1 cm
    public static final ProfiledPIDController PID =
            new ProfiledPIDController(P_GAIN, I_GAIN, D_GAIN, CONSTRAINTS);

    static {
        PID.setTolerance(TOLERANCE);
    }

    public static final double GEAR_RATIO = 36;
    public static final double SPROCKET_CIRCUMFERENCE = 2D * Math.PI * Units.inchesToMeters(0.943);
    public static final double PULLY_RATIO = 2;
    public static final double POS_CONV_FACTOR =
            (1D / GEAR_RATIO) * SPROCKET_CIRCUMFERENCE * PULLY_RATIO; // meters
    public static final double VEL_CONV_FACTOR = POS_CONV_FACTOR / 60D; // mps

    public static final double MIN_HEIGHT = 0;
    // Hi welcome back future Jeff, yes you did in fact change the sprocket radius and then
    // multiplied the (new / old) conversion factors with the old max.
    // If it didn't work, thats why your back here; hello and fuck you!
    public static final double MAX_HEIGHT = 1.355;
}
