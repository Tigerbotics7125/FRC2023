/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import io.github.tigerbotics7125.robot.commands.SuperStructure;
import io.github.tigerbotics7125.robot.subsystem.Arm2;
import io.github.tigerbotics7125.robot.subsystem.Elevator2;
import io.github.tigerbotics7125.robot.subsystem.Intake;
import io.github.tigerbotics7125.robot.subsystem.Wrist2;
import java.util.List;
import java.util.Map;

public class StandstillCube implements Auto {

    private Elevator2 mElev;
    private Arm2 mArm;
    private Wrist2 mWrist;
    private Intake mIntake;

    public StandstillCube(Elevator2 elev, Arm2 arm, Wrist2 wrist, Intake intake) {
        mElev = elev;
        mArm = arm;
        mWrist = wrist;
        mIntake = intake;
    }

    @Override
    public CommandBase preCommand() {
        return SuperStructure.highCube(mElev, mArm, mWrist, mIntake)
                .andThen(Commands.waitSeconds(3))
                .andThen(mIntake.outakeRoutine())
                .andThen(mIntake.grippersOpen())
                .andThen(Commands.waitSeconds(2))
                .andThen(SuperStructure.home(mElev, mArm, mWrist, mIntake));
    }

    @Override
    public List<PathPlannerTrajectory> getPath() {
        return List.of();
    }

    @Override
    public Map<String, Command> getEventMap() {
        return Map.of();
    }
}
