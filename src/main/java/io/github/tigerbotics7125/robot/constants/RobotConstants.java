/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class RobotConstants {

    public static final double NOMINAL_VOLTAGE = 12.0;

    public static final double ROBOT_WIDTH_METERS = Units.inchesToMeters(32);
    public static final double ROBOT_LENGTH_METERS = Units.inchesToMeters(38);

    public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;

    public static final double NEO_FREE_SPEED_RPM = 5676;
}
