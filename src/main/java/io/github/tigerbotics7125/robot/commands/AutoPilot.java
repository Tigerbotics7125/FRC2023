/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.commands;

import static io.github.tigerbotics7125.robot.constants.AutoPilotConstants.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import io.github.tigerbotics7125.robot.OperatorInterface;
import io.github.tigerbotics7125.robot.OperatorInterface.OpZone;
import io.github.tigerbotics7125.robot.constants.AutoPilotConstants;
import io.github.tigerbotics7125.robot.constants.RobotConstants;
import io.github.tigerbotics7125.robot.constants.field.FieldArea;
import io.github.tigerbotics7125.robot.constants.field.FieldZone;
import io.github.tigerbotics7125.robot.constants.field.GridLocation.Column;
import io.github.tigerbotics7125.robot.subsystem.Drivetrain;
import java.util.Set;

public class AutoPilot implements Command {
    private final Field2d mDbgField;
    private final FieldObject2d mTrajectoryPath;

    private final Drivetrain mDrivetrain;
    private final PathConstraints mPathConstraints;

    private final Timer mTimer;

    private PathPlannerTrajectory mTrajectory;
    private final PPHolonomicDriveController mController;

    public AutoPilot(Drivetrain drivetrain) {
        mDbgField = new Field2d();
        mTrajectoryPath = mDbgField.getObject("trajectoryPath");

        mDrivetrain = drivetrain;
        mPathConstraints = new PathConstraints(MAX_VELOCITY_MPS, MAX_ACCELERATION_MPSPS);

        mTimer = new Timer();
        mTrajectory = new PathPlannerTrajectory();
        mController = new PPHolonomicDriveController(X_PID, Y_PID, THETA_PID);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(mDrivetrain);
    }

    @Override
    public void initialize() {
        mTrajectory = generatePath();
        mTrajectoryPath.setPoses(mTrajectory.getStates().stream().map(s -> s.poseMeters).toList());

        // clear timer
        mTimer.reset();
        mTimer.start();

        // Update path planner app of what we are doing.
        PathPlannerServer.sendActivePath(mTrajectory.getStates());

        // TODO: revamp field constant naming, its weird.
        FieldArea area;
        if (OperatorInterface.getZone().equals(OpZone.SUBSTATIONS)) {
            area = FieldArea.LOADING_ZONE;
        } else {
            area = FieldArea.COMMUNITY;
        }
        // If the area where we want to drive to does not contain where we currently are, ie: we are
        // in the battlezone, we probably shouldn't use autopilot
        if (!area.contains(mDrivetrain.getPose())) {
            end(true);
        }
    }

    @Override
    public void execute() {

        // If the previous iteration has completed determine if we need to do it again.
        if (mTimer.hasElapsed(mTrajectory.getTotalTimeSeconds())) {
            Pose2d drive = mDrivetrain.getPose();
            Pose2d ideal = mTrajectory.getEndState().poseMeters;
            Translation2d drivePose = drive.getTranslation();
            Translation2d idealPose = ideal.getTranslation();
            Rotation2d driveRotation = drive.getRotation();
            Rotation2d idealRotation = ideal.getRotation();
            // If drive is > 3cm away from pose, or > 2 deg out of allignment, iterate and do it
            // again.
            double poseDifferenceMeters = idealPose.getDistance(drivePose);
            double rotationDifferenceDegrees =
                    idealRotation.rotateBy(driveRotation.unaryMinus()).getDegrees();
            if (poseDifferenceMeters > .03 || rotationDifferenceDegrees > 2) {
                initialize();
            } else {
                end(false);
                return;
            }
        }

        double currentTime = mTimer.get();
        PathPlannerState desiredState = (PathPlannerState) mTrajectory.sample(currentTime);

        Pose2d currentPose = mDrivetrain.getPose();
        PathPlannerServer.sendPathFollowingData(
                new Pose2d(
                        desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation),
                currentPose);

        ChassisSpeeds targetSpeeds = mController.calculate(currentPose, desiredState);

        mDrivetrain.setChassisSpeeds(targetSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        mTimer.stop();

        // Clear traj visualization.
        mTrajectory = new PathPlannerTrajectory();

        // stop movement from perpetuating.
        mDrivetrain.setChassisSpeeds(new ChassisSpeeds());
    }

    public PathPlannerTrajectory generatePath() {
        Pose2d initialPose = mDrivetrain.getPose();
        Transform2d robotSpacing =
                new Transform2d(
                        new Translation2d(-RobotConstants.ROBOT_WIDTH_METERS / 2.0, 0),
                        new Rotation2d());
        Pose2d finalPose = getDesiredPose().transformBy(robotSpacing);

        // trajectory angle to leave and arrive from
        Rotation2d heading =
                new Rotation2d(
                                initialPose.getX() - finalPose.getX(),
                                initialPose.getY() - finalPose.getY())
                        .rotateBy(Rotation2d.fromDegrees(180));

        ChassisSpeeds speeds = mDrivetrain.getChassisSpeeds();
        // going slow is defined as 1/4 m/s TODO: make constant
        boolean goingSlow =
                Math.abs(speeds.vxMetersPerSecond) <= .25
                        && Math.abs(speeds.vyMetersPerSecond) <= .25;

        PathPoint initialPoint =
                goingSlow
                        ? new PathPoint(
                                initialPose.getTranslation(), heading, initialPose.getRotation())
                        : PathPoint.fromCurrentHolonomicState(initialPose, speeds);
        PathPoint finalPoint =
                new PathPoint(finalPose.getTranslation(), heading, finalPose.getRotation());

        PathPlannerTrajectory traj =
                PathPlanner.generatePath(mPathConstraints, initialPoint, finalPoint);
        // Put path on dbg field.
        mTrajectoryPath.setPoses(traj.getStates().stream().map(s -> s.poseMeters).toList());

        return traj;
    }

    public Pose2d getDesiredPose() {
        // System.out.println(OperatorInterface.getSelectedNode()[0] % 3);
        Column desiredColumn =
                switch (OperatorInterface.getSelectedNode()[0] % 3) {
                    case 0 -> Column.LEFT_CONE;
                    case 1 -> Column.MID_CUBE;
                    case 2 -> Column.RIGHT_CONE;
                    default -> Column.MID_CUBE;
                };

        // System.out.println(OperatorInterface.getZone());
        FieldZone desiredZone =
                switch (OperatorInterface.getZone()) {
                    case NODES -> switch (OperatorInterface.getSelectedNode()[0]) {
                        case 0, 1, 2 -> FieldZone.LEFT_GRID;
                        case 3, 4, 5 -> FieldZone.CO_OP_GRID;
                        case 6, 7, 8 -> FieldZone.RIGHT_GRID;
                        default -> FieldZone.CO_OP_GRID;
                    };
                    case SUBSTATIONS -> switch (OperatorInterface.getSelectedSubstation()) {
                        case 0 -> FieldZone.DOUBLE_SUBSTATION_LEFT;
                        case 1 -> FieldZone.DOUBLE_SUBSTATION_RIGHT;
                        default -> FieldZone.DOUBLE_SUBSTATION_LEFT;
                    };
                    default -> FieldZone.CO_OP_GRID;
                };

        // System.out.println(desiredZone.name() + " " + desiredColumn.name());
        return AutoPilotConstants.getAutoPilotPose(desiredZone, desiredColumn);
    }

    public PathPlannerTrajectory getTrajectory() {
        return mTrajectory;
    }
}
