/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import io.github.tigerbotics7125.robot.subsystem.Arm2;
import io.github.tigerbotics7125.robot.subsystem.Arm2.ArmState;
import io.github.tigerbotics7125.robot.subsystem.Elevator2;
import io.github.tigerbotics7125.robot.subsystem.Elevator2.ElevState;
import io.github.tigerbotics7125.robot.subsystem.Intake;
import io.github.tigerbotics7125.robot.subsystem.Wrist2;
import io.github.tigerbotics7125.robot.subsystem.Wrist2.WristState;

/** superStructure */
public class SuperStructure {

    private static boolean isInStartConfig = true;

    public static CommandBase home(Elevator2 elev, Arm2 arm, Wrist2 wrist, Intake intake) {
        return intake.grippersClose()
                .andThen(arm.setState(ArmState.STOW))
                .andThen(Commands.waitSeconds(.3))
                .andThen(wrist.setState(WristState.STOW))
                .andThen(Commands.waitSeconds(.3))
                .andThen(elev.setState(ElevState.HOME))
                .andThen(intake.grippersOpen());
    }

    // Gets the super struc out of start config.
    private static CommandBase ensureSafe(Elevator2 elev, Arm2 arm, Wrist2 wrist, Intake intake) {
        return Commands.either(
                home(elev, arm, wrist, intake).andThen(() -> isInStartConfig = false),
                Commands.none(),
                () -> isInStartConfig);
    }

    public static CommandBase groundIntake(Elevator2 elev, Arm2 arm, Wrist2 wrist, Intake intake) {
        return ensureSafe(elev, arm, wrist, intake)
                .andThen(elev.setState(ElevState.GROUND_INTAKE))
                .andThen(arm.setState(ArmState.GROUND_INTAKE))
                .andThen(wrist.setState(WristState.GROUND_INTAKE));
    }

    public static CommandBase midCube(Elevator2 elev, Arm2 arm, Wrist2 wrist, Intake intake) {
        return ensureSafe(elev, arm, wrist, intake)
                .andThen(elev.setState(ElevState.MID_CUBE))
                .andThen(arm.setState(ArmState.MID_CUBE))
                .andThen(wrist.setState(WristState.MID_CUBE));
    }

    public static CommandBase highCube(Elevator2 elev, Arm2 arm, Wrist2 wrist, Intake intake) {
        return ensureSafe(elev, arm, wrist, intake)
                .andThen(arm.setState(ArmState.HIGH_CUBE))
                .andThen(wrist.setState(WristState.HIGH_CUBE))
                .andThen(elev.setState(ElevState.HIGH_CUBE));
    }
}
