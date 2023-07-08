/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.Map;

public interface Auto {

    default CommandBase preCommand() {
        return Commands.none();
    }
    ;

    List<PathPlannerTrajectory> getPath();

    Map<String, Command> getEventMap();

    default CommandBase postCommand() {
        return Commands.none();
    }
}
