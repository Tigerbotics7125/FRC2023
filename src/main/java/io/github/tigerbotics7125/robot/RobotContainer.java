/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import static io.github.tigerbotics7125.robot.constants.OIConstants.*;
import static io.github.tigerbotics7125.robot.constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import io.github.tigerbotics7125.robot.subsystem.Drivetrain;
import io.github.tigerbotics7125.robot.subsystem.Drivetrain.TurningMode;
import io.github.tigerbotics7125.robot.subsystem.Vision;
import io.github.tigerbotics7125.robot.sysid.SysIdMechanism;
import io.github.tigerbotics7125.robot.sysid.logging.SysIdDrivetrainLogger;
import io.github.tigerbotics7125.robot.sysid.logging.SysIdGeneralMechanismLogger;
import io.github.tigerbotics7125.tigerlib.input.controller.XboxController;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;

public class RobotContainer {

    XboxController mDriver = new XboxController(kDriverPort);
    XboxController mOperator = new XboxController(kOperatorPort);
    Trigger mRioUserButton = new Trigger(RobotController::getUserButton);

    Drivetrain mDrivetrain = new Drivetrain();
    Vision mVision = new Vision();

    boolean sysIdActive = true;
    SysIdGeneralMechanismLogger sysIdMechLogger;
    SysIdDrivetrainLogger sysIdDriveLogger;
    SendableChooser<SubsystemBase> sysIdMechChooser;

    // AutoPilot mAutoPilot = new AutoPilot(mDrivetrain);

    Field2d mField = new Field2d();

    public RobotContainer() {
        configureTriggers();

        mDrivetrain.setName("Drivetrain");
        mDrivetrain.setDefaultCommand(
                mDrivetrain.drive(
                        mDriver.leftY()::get,
                        mDriver.leftX()::get,
                        mDriver.rightX()::get,
                        mDriver.rightY()::get));

        sysIdMechChooser = new SendableChooser<>();
        sysIdMechChooser.setDefaultOption(mDrivetrain.getName(), mDrivetrain);
        Shuffleboard.getTab("sysId").add(sysIdMechChooser);

        if (Robot.isSimulation()) {
            mField.getObject("AprilTags")
                    .setPoses(AprilTagLayout.getTags().stream().map(Pose3d::toPose2d).toList());
        }
    }

    private void configureTriggers() {
        // Generic triggers

        mDriver.b().trigger(mDrivetrain::resetGyro);
        mDriver.lb().trigger(() -> mDrivetrain.setTurningMode(TurningMode.JOYSTICK_DIRECT));
        mDriver.rb().trigger(() -> mDrivetrain.setTurningMode(TurningMode.JOYSTICK_ANGLE));
        mDriver.y().trigger(() -> mDrivetrain.setTurningMode(TurningMode.HEADING_LOCK));

        // mDriver.start().debounce(.02).activate(ActivationCondition.WHILE_HIGH).trigger(mAutoPilot);
    }

    public Command getAutonomousCommand() {
        if (sysIdActive) {
            if (!sysIdMechChooser.getSelected().getName().equals(mDrivetrain.getName())) {
                // Not drivetrain
                sysIdMechLogger = new SysIdGeneralMechanismLogger();
                Subsystem sysIdMechSubsystem = sysIdMechChooser.getSelected();
                SysIdMechanism sysIdMech = (SysIdMechanism) sysIdMechSubsystem;
                return Commands.runOnce(sysIdMechLogger::initLogging, sysIdMechSubsystem)
                        .andThen(
                                Commands.run(
                                        () -> {
                                            sysIdMechLogger.log(
                                                    sysIdMechLogger.measureVoltage(
                                                            List.of(sysIdMech.getMotor())),
                                                    sysIdMech.getPosition(),
                                                    sysIdMech.getVelocity());
                                            sysIdMechLogger.setMotorControllers(
                                                    sysIdMechLogger.getMotorVoltage(),
                                                    List.of(sysIdMech.getMotor()));
                                        },
                                        sysIdMechSubsystem));
            } else {
                // drivetrain
                sysIdDriveLogger = new SysIdDrivetrainLogger();
                Subsystem sysIdDriveSubsystem = sysIdMechChooser.getSelected();
                return Commands.runOnce(sysIdDriveLogger::initLogging, sysIdDriveSubsystem)
                        .andThen(
                                new NotifierCommand(
                                        () -> {
                                            sysIdDriveLogger.log(
                                                    sysIdDriveLogger.measureVoltage(
                                                            mDrivetrain.getLeftMotors()),
                                                    sysIdDriveLogger.measureVoltage(
                                                            mDrivetrain.getRightMotors()),
                                                    mDrivetrain.getWheelPositions().frontLeftMeters,
                                                    mDrivetrain.getWheelPositions()
                                                            .frontRightMeters,
                                                    mDrivetrain.getWheelSpeeds()
                                                            .frontLeftMetersPerSecond,
                                                    mDrivetrain.getWheelSpeeds()
                                                            .frontRightMetersPerSecond,
                                                    mDrivetrain.getHeading().getDegrees(),
                                                    mDrivetrain.getGyroRate());
                                            sysIdDriveLogger.setMotorControllers(
                                                    sysIdDriveLogger.getLeftMotorVoltage(),
                                                    mDrivetrain.getLeftMotors());
                                            sysIdDriveLogger.setMotorControllers(
                                                    sysIdDriveLogger.getRightMotorVoltage(),
                                                    mDrivetrain.getRightMotors());
                                        },
                                        0.005,
                                        mDrivetrain)).handleInterrupt(sysIdDriveLogger::sendData);
            }
        } else {
            return Commands.print("No auto selected!");
        }
    }

    /** Periodic call, always runs. */
    public void periodic() {
        // mAutoPilot.generatePath();

        // mField.setRobotPose(mDrivetrain.getPose());
        SmartDashboard.putData(mField);

        List<Pose2d> trajPoses = new ArrayList<>();
        // DS always on trajectory
        // mAutoPilot.getTrajectory().getStates().forEach((state) ->
        // trajPoses.add(state.poseMeters));
        mField.getObject("AutoPilot_GenPath").setPoses(trajPoses);
        // Active path.
        trajPoses.clear();
        /*

        mAutoPilot
                .getActiveTrajectory()
                .getStates()
                .forEach((state) -> trajPoses.add(state.poseMeters));
        mField.getObject("AutoPilot_ActivePath").setPoses(trajPoses);

        mField.getObject("CHARGING_STATION").setPoses(FieldArea.CHARGING_STATION.getPoses());
        mField.getObject("LOADING_ZONE").setPoses(FieldArea.LOADING_ZONE.getPoses());
        mField.getObject("COMMUNITY").setPoses(FieldArea.COMMUNITY.getPoses());

        SmartDashboard.putBoolean(
                "WithinCommunity", FieldArea.COMMUNITY.contains(mDrivetrain.getPose()));
        */
    }

    /** Periodic call, only runs during simulation. */
    public void simulationPeriodic() {
        // update sim vision with our robot pose.
        // mVision.updateCameraPose(mDrivetrain.getPose());

        List<PhotonTrackedTarget> targets = new ArrayList<>(mVision.getTargets());
        if (targets.size() > 0) {
            mField.getObject("SeenTags")
                    .setPoses(
                            targets.stream()
                                    .map(
                                            (target) ->
                                                    AprilTagLayout.getTagPose(
                                                                    target.getFiducialId())
                                                            .toPose2d())
                                    .toList());
            mField.getObject("BestTag")
                    .setPose(
                            AprilTagLayout.getTagPose(
                                            mVision.getBestTag(kSortingMode, kAmbiguityThreshold)
                                                    .getFiducialId())
                                    .toPose2d());
        }
    }
}
