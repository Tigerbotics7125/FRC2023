/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.constants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;

public class SuperStructureConstants {
    public static final int ELEVATOR_HEIGHT_SENSOR_CHANNEL = 0;
    public static final AnalogInput ELEVATOR_HEIGHT_SENSOR_RAW = new AnalogInput(1);
    public static final int ELEVATOR_MASTER_ID = 11;
    public static final int ELEVATOR_SLAVE_ID = 12;

    public static final int ARM_MOTOR_ID = 21;

    public static final int WRIST_MOTOR_ID = 31;

    public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
}
