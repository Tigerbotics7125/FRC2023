/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import static io.github.tigerbotics7125.robot.constants.OIConstants.*;
import static io.github.tigerbotics7125.robot.constants.RobotConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.tigerbotics7125.robot.commands.AutoPilot;
import io.github.tigerbotics7125.robot.constants.field.FieldArea;
import io.github.tigerbotics7125.robot.subsystem.Drivetrain;
import io.github.tigerbotics7125.robot.subsystem.Drivetrain.TurningMode;
import io.github.tigerbotics7125.robot.subsystem.Intake;
import io.github.tigerbotics7125.robot.subsystem.Vision;
import io.github.tigerbotics7125.robot.sysid.logging.SysIdDrivetrainLogger;
import io.github.tigerbotics7125.robot.sysid.logging.SysIdGeneralMechanismLogger;
import io.github.tigerbotics7125.tigerlib.input.controller.XboxController;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger.ActivationCondition;
import java.util.ArrayList;
import java.util.List;

public class RobotContainer {

    XboxController mDriver = new XboxController(DRIVER_CONTROLLER_PORT);
    XboxController mOperator = new XboxController(OPERATOR_CONTROLLER_PORT);
    Trigger mRioUserButton = new Trigger(RobotController::getUserButton);

    Vision mVision = new Vision();
    Drivetrain mDrivetrain = new Drivetrain();
    Intake mIntake = new Intake();
    // SuperStructure mSuperStruc = new SuperStructure();
    Compressor mCompressor = new Compressor(PNEUMATICS_MODULE_TYPE);

    boolean sysIdActive = false;
    SysIdGeneralMechanismLogger sysIdMechLogger;
    SysIdDrivetrainLogger sysIdDriveLogger;
    SendableChooser<SubsystemBase> sysIdMechChooser;

    AutoPilot mAutoPilot = new AutoPilot(mDrivetrain);
    Field2d mAutoPilotField = new Field2d();

    Field2d mField = new Field2d();

    public RobotContainer() {
        mCompressor.enableDigital();
        configTriggers();
        configDashboard();
        configDefaultCommands();

        sysIdMechChooser = new SendableChooser<>();
        sysIdMechChooser.setDefaultOption(mDrivetrain.getName(), mDrivetrain);
        Shuffleboard.getTab("sysId").add(sysIdMechChooser);

        if (Robot.isSimulation()) {
            mField.getObject("AprilTags")
                    .setPoses(AprilTagLayout.getTags().stream().map(Pose3d::toPose2d).toList());
        }
    }

    private void configTriggers() {
        // Driver
        mDriver.b().trigger(mDrivetrain::resetGyro);
        mDriver.lb().trigger(() -> mDrivetrain.setTurningMode(TurningMode.JOYSTICK_DIRECT));
        mDriver.rb().trigger(() -> mDrivetrain.setTurningMode(TurningMode.JOYSTICK_ANGLE));
        mDriver.y().trigger(() -> mDrivetrain.setTurningMode(TurningMode.HEADING_LOCK));

        mDriver.start().activate(ActivationCondition.WHILE_HIGH).trigger(mAutoPilot);

        // Operator
        mOperator.rb().trigger(mIntake.grabObject());
        mOperator.lb().trigger(mIntake.releaseObject());

        mOperator.pov.left().trigger(OperatorInterface::selectLeft);
        mOperator.pov.up().trigger(OperatorInterface::selectUp);
        mOperator.pov.right().trigger(OperatorInterface::selectRight);
        mOperator.pov.down().trigger(OperatorInterface::selectDown);

        // Subsystems

    }

    public void configDashboard() {
        Shuffleboard.getTab("odometry").add(mField);

        ShuffleboardTab autoPilotTab = Shuffleboard.getTab("AutoPilot");

        autoPilotTab.addBoolean(
                "WithinCommunity", () -> FieldArea.COMMUNITY.contains(mDrivetrain.getPose()));
        autoPilotTab.add(mAutoPilotField);
    }

    private void configDefaultCommands() {
        mDrivetrain.setDefaultCommand(
                mDrivetrain.driveCmd(
                        mDriver.leftY()::get,
                        mDriver.leftX()::get,
                        mDriver.rightY()::get,
                        mDriver.rightX()::get,
                        true));
    }

    // sysid command... ew
    /*
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
                                        mDrivetrain))
                        .handleInterrupt(sysIdDriveLogger::sendData);
            }
        } else {
            return Commands.print("No auto selected!");
        }
    }
    */

    /** Periodic call, always runs. */
    public void periodic() {
        mAutoPilot.generatePath();

        mVision.getEstimatedPose().ifPresent(mDrivetrain::addVisionEstimate);

        mField.setRobotPose(mDrivetrain.getPose());
        mAutoPilotField.setRobotPose(mDrivetrain.getPose());

        List<Pose2d> trajPoses = new ArrayList<>();
        // DS always on trajectory
        mAutoPilot.getTrajectory().getStates().forEach((state) -> trajPoses.add(state.poseMeters));
        mAutoPilotField.getObject("GenPath").setPoses(trajPoses);
        // Active path.
        trajPoses.clear();

        mAutoPilot
                .getActiveTrajectory()
                .getStates()
                .forEach((state) -> trajPoses.add(state.poseMeters));
        mAutoPilotField.getObject("ActivePath").setPoses(trajPoses);

        mAutoPilotField
                .getObject("CHARGING_STATION")
                .setPoses(FieldArea.CHARGING_STATION.getPoses());
        mAutoPilotField.getObject("LOADING_ZONE").setPoses(FieldArea.LOADING_ZONE.getPoses());
        mAutoPilotField.getObject("COMMUNITY").setPoses(FieldArea.COMMUNITY.getPoses());
    }

    /** Periodic call, only runs during simulation. */
    public void simulationPeriodic() {
        // update sim vision with our robot pose.
        mVision.update(mDrivetrain.getPose());
    }
}
