/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.Map;

public class CenterDelayMobility implements Auto {

    @Override
    public List<PathPlannerTrajectory> getPath() {
        return PathPlanner.loadPathGroup("CenterDelayMobility", new PathConstraints(2, 1));
    }

    @Override
    public Map<String, Command> getEventMap() {
        return Map.of("Delay", Commands.waitSeconds(7));
    }
}
