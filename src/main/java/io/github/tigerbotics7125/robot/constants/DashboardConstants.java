/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.constants;

import edu.wpi.first.wpilibj.shuffleboard.*;

public class DashboardConstants {

    public static final ShuffleboardTab MAIN_TAB = Shuffleboard.getTab("Match Play");
    public static final ShuffleboardTab SUPER_STRUC_TAB = Shuffleboard.getTab("Super Structure");
    public static final ShuffleboardTab ODO_VISION_TAB = Shuffleboard.getTab("Odometry");

    public static final String UNSELECTED_CONE_COLOR = "#40361c";
    public static final String SELECTED_CONE_COLOR = "#FFD871";
    public static final String UNSELECTED_CUBE_COLOR = "#251b40";
    public static final String SELECTED_CUBE_COLOR = "#976eff";
    public static final String UNSELECTED_HYBRID_COLOR = "#403339";
    public static final String SELECTED_HYBRID_COLOR = "#CBA3B8";
    public static final String UNSELECTED_SUBSTATION_COLOR = "#505050";
    public static final String SELECTED_SUBSTATION_COLOR = "FFFFFF";
}
