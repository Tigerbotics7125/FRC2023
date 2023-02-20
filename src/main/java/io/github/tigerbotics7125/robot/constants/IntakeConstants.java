/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.constants;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IntakeConstants {

    // Default Characteristics
    public static final DoubleSolenoid.Value DEFAULT_PNEUMATIC_STATE = Value.kReverse;

    // IDs
    public static final int MASTER_ID = 11;
    public static final int SLAVE_ID = 12;
    public static final int FORWARDS_CHANNEL = 0;
    public static final int REVERSE_CHANNEL = 1;
    public static final int OCCUPANCY_1_PORT = 1;
    public static final int OCCUPANCY_2_PORT = 2;

    // Occupancy settings
    public static final double OCCUPANCY_VOLTAGE_THRESHOLD = 2.5;
    public static final double OCCUPANCY_DEBOUNCE_TIME = .2; // seconds

    // Characteristics
    public static final float STALL_TORQUE_NEWTON_METERS = 19.40175484f;
    public static final float FREE_SPEED_RPM = 550.0f;

    // Motor values
    public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
    public static final IdleMode IDLE_MODE = IdleMode.kCoast;
    public static final int STALL_CURRENT_LIMIT_AMPS = 8;
    public static final int FREE_SPEED_CURRENT_LIMIT_AMPS = 2;
    public static final double OPEN_LOOP_RAMP_RATE = 0.05;
}
