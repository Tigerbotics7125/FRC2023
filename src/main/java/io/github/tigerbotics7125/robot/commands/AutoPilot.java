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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import io.github.tigerbotics7125.robot.constants.AutoPilotConstants;
import io.github.tigerbotics7125.robot.constants.RobotConstants;
import io.github.tigerbotics7125.robot.constants.field.FieldArea;
import io.github.tigerbotics7125.robot.constants.field.FieldZone;
import io.github.tigerbotics7125.robot.constants.field.GridLocation.Column;
import io.github.tigerbotics7125.robot.subsystem.Drivetrain;
import java.util.Set;

public class AutoPilot implements Command {
    private SendableChooser<FieldZone> mZoneChooser;
    private SendableChooser<Column> mGridChooser;
    private final Field2d mDbgField;
    private final FieldObject2d mPathFollwing;
    private final FieldObject2d mPathGen;

    private final Drivetrain mDrivetrain;
    private final PathConstraints mPathConstraints;

    private final Timer mTimer;
    // The trajectory that gets followed
    private PathPlannerTrajectory mTrajFollowing;
    // The trajectory always being generated for DS view.
    private PathPlannerTrajectory mTrajGen;
    private final PPHolonomicDriveController mController;

    public AutoPilot(Drivetrain drivetrain) {
        mZoneChooser = new SendableChooser<>();
        mGridChooser = new SendableChooser<>();
        SmartDashboard.putData(mZoneChooser);
        SmartDashboard.putData(mGridChooser);
        mZoneChooser.setDefaultOption(FieldZone.LEFT_GRID.name(), FieldZone.LEFT_GRID);
        for (var zone : FieldZone.values()) {
            mZoneChooser.addOption(zone.name(), zone);
        }
        mGridChooser.setDefaultOption(Column.MID_CUBE.name(), Column.MID_CUBE);
        mGridChooser.addOption(Column.LEFT_CONE.name(), Column.LEFT_CONE);
        mGridChooser.addOption(Column.RIGHT_CONE.name(), Column.RIGHT_CONE);
        mDbgField = new Field2d();
        mPathFollwing = mDbgField.getObject("pathFollowing");
        mPathGen = mDbgField.getObject("pathGen");

        mDrivetrain = drivetrain;
        mPathConstraints = new PathConstraints(kMaxVelocity, kMaxAcceleration);

        mTimer = new Timer();
        mTrajGen = mTrajFollowing = new PathPlannerTrajectory();
        mController = new PPHolonomicDriveController(kXController, kYController, kZController);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(mDrivetrain);
    }

    @Override
    public void initialize() {
        mTrajFollowing = mTrajGen;
        mPathFollwing.setPoses(mTrajFollowing.getStates().stream().map(s -> s.poseMeters).toList());

        mTimer.reset();
        mTimer.start();

        PathPlannerServer.sendActivePath(mTrajFollowing.getStates());
    }

    @Override
    public void execute() {
        FieldZone selectedZone = mZoneChooser.getSelected();
        FieldArea area = FieldArea.COMMUNITY;
        if (selectedZone == FieldZone.DOUBLE_SUBSTATION_LEFT
                || selectedZone == FieldZone.DOUBLE_SUBSTATION_RIGHT) area = FieldArea.LOADING_ZONE;
        // if the robot is not within the target area, continue running default command (ie: let
        // driver drive lol)
        if (!area.contains(mDrivetrain.getPose())) {
            mDrivetrain.getDefaultCommand().execute();
            initialize();
            return;
        }

        if (mTimer.hasElapsed(mTrajFollowing.getTotalTimeSeconds())) {
            // restart command.
            initialize();
        }

        double currentTime = mTimer.get();
        PathPlannerState desiredState = (PathPlannerState) mTrajFollowing.sample(currentTime);

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
        mTrajFollowing = new PathPlannerTrajectory();

        // stop movement from perpetuating.
        mDrivetrain.setChassisSpeeds(new ChassisSpeeds());
    }

    public void generatePath() {
        Pose2d initialPose = mDrivetrain.getPose();
        Transform2d robotSpacing =
                new Transform2d(
                        new Translation2d(-RobotConstants.kRobotWidthMeters / 2.0, 0),
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

        mTrajGen = PathPlanner.generatePath(mPathConstraints, initialPoint, finalPoint);

        // Put path on dbg field.
        mPathGen.setPoses(mTrajGen.getStates().stream().map(s -> s.poseMeters).toList());
    }

    public Pose2d getDesiredPose() {
        return AutoPilotConstants.getAutoPilotPose(
                mZoneChooser.getSelected(), mGridChooser.getSelected());
    }

    public PathPlannerTrajectory getActiveTrajectory() {
        return mTrajFollowing;
    }

    public PathPlannerTrajectory getTrajectory() {
        return mTrajGen;
    }
}
