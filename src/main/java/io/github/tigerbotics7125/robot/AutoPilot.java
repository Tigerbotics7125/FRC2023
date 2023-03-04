/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import io.github.tigerbotics7125.lib.AllianceFlipUtil;
import io.github.tigerbotics7125.robot.commands.AutoPilotCmd;
import io.github.tigerbotics7125.robot.constants.AutoPilotConstants;
import io.github.tigerbotics7125.robot.constants.AutoPilotConstants.AutoPilotPoint;
import io.github.tigerbotics7125.robot.subsystem.Drivetrain;
import java.util.function.Supplier;

/**
 * This class serves as a helper to generate auto navigation and subsystem activation commands.
 *
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class AutoPilot {

    private final Drivetrain mDrivetrain;
    private final Supplier<AutoPilotPoint> mDestinationSupplier;

    public AutoPilot(Drivetrain drivetrain, Supplier<AutoPilotPoint> destinationSupplier) {
        mDrivetrain = drivetrain;
        mDestinationSupplier = destinationSupplier;
    }

    /**
     * @param point The point to drive to.
     * @return A Command which will drive the robot to the given point; or the default drive command
     *     if the path is not viable.
     */
    public Command getAutoPilotCommand() {

        Command follower =
                new AutoPilotCmd(
                        mDrivetrain, () -> generateAutoPilotTrajectory(mDestinationSupplier.get()));
        Command disable = Commands.runOnce(mDrivetrain::disable);

        var cmd =
                follower.andThen(disable)
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
                        .withName("AutoPilot");
        // autoNavCmd.until(() -> !AutoPilot.isViable(mDrivetrain.getPose(), point));
        cmd.addRequirements(mDrivetrain);

        return cmd;
    }

    /**
     * @param drive The drivetrani used in the command.
     * @param point The point to drive to.
     * @return A trajectory from where the robot is currently, to the given point.
     */
    public PathPlannerTrajectory generateAutoPilotTrajectory(AutoPilotPoint point) {
        Translation2d pose = point.getPose();
        Rotation2d rot =
                switch (point) {
                    case SINGLE_SUBSTATION -> new Rotation2d(3.0 * Math.PI / 2.0);
                    case DOUBLE_SUBSTATION_BARRIER, DOUBLE_SUBSTATION_FAR -> new Rotation2d(
                            Math.PI);
                    default -> new Rotation2d(); // all nodes.
                };

        pose = AllianceFlipUtil.apply(pose);
        rot = AllianceFlipUtil.apply(rot);

        PathPoint initialPoint =
                PathPoint.fromCurrentHolonomicState(
                        mDrivetrain.getPose(), mDrivetrain.getChassisSpeeds());
        PathPoint finalPoint = new PathPoint(pose, rot.rotateBy(new Rotation2d(180)), rot);
        PathConstraints constraints =
                new PathConstraints(
                        AutoPilotConstants.MAX_VELOCITY_MPS,
                        AutoPilotConstants.MAX_ACCELERATION_MPSPS);

        return PathPlanner.generatePath(constraints, initialPoint, finalPoint);
    }

    /**
     * @param robotPose The pose of the robot.
     * @param targetPoint The target point of the desired AutoPilot sequence.
     * @return
     */
    public static boolean isViable(Pose2d robotPose, AutoPilotPoint targetPoint) {
        return targetPoint.mBoundary.confines(AllianceFlipUtil.apply(robotPose));
    }
}
