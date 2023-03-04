/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import java.util.Map;

public class None implements Auto {

    @Override
    public List<PathPlannerTrajectory> getPath() {
        return List.of();
    }

    @Override
    public Map<String, Command> getEventMap() {
        return Map.of();
    }
}
