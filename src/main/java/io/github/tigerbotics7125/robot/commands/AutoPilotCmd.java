/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.commands;

import static io.github.tigerbotics7125.robot.constants.AutoPilotConstants.*;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import io.github.tigerbotics7125.robot.subsystem.Drivetrain;
import java.util.Set;
import java.util.function.Supplier;

public class AutoPilotCmd implements Command {

    private final Drivetrain mDrivetrain;
    private final Supplier<PathPlannerTrajectory> mTrajSupplier;

    private final Timer mTimer = new Timer();
    private final PPHolonomicDriveController mController =
            new PPHolonomicDriveController(X_PID, Y_PID, THETA_PID);

    private PathPlannerTrajectory mTraj;

    public AutoPilotCmd(Drivetrain drivetrain, Supplier<PathPlannerTrajectory> trajSupplier) {
        mDrivetrain = drivetrain;
        mTrajSupplier = trajSupplier;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(mDrivetrain);
    }

    @Override
    public void initialize() {
        mTraj = mTrajSupplier.get();

        mTimer.reset();
        mTimer.start();
    }

    @Override
    public void execute() {
        if (mTimer.hasElapsed(mTraj.getTotalTimeSeconds())) {
            end(false);
            return;
        }

        double currentTime = mTimer.get();
        PathPlannerState desiredState = (PathPlannerState) mTraj.sample(currentTime);

        Pose2d currentPose = mDrivetrain.getPose();

        ChassisSpeeds targetSpeeds = mController.calculate(currentPose, desiredState);
        mDrivetrain.setChassisSpeeds(targetSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        mTimer.stop();
    }
}
