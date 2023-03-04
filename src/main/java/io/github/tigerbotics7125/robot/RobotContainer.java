/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import static io.github.tigerbotics7125.robot.constants.OIConstants.*;
import static io.github.tigerbotics7125.robot.constants.RobotConstants.PNEUMATICS_MODULE_TYPE;
import static io.github.tigerbotics7125.tigerlib.input.trigger.Trigger.ActivationCondition.*;

import com.pathplanner.lib.auto.MecanumAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import io.github.tigerbotics7125.lib.AllianceFlipUtil;
import io.github.tigerbotics7125.robot.auto.Auto;
import io.github.tigerbotics7125.robot.auto.CenterDelayMobility;
import io.github.tigerbotics7125.robot.auto.None;
import io.github.tigerbotics7125.robot.constants.AutoPilotConstants;
import io.github.tigerbotics7125.robot.subsystem.Drivetrain;
import io.github.tigerbotics7125.robot.subsystem.Drivetrain.TurningMode;
import io.github.tigerbotics7125.robot.subsystem.Intake;
import io.github.tigerbotics7125.robot.subsystem.SuperStructure;
import io.github.tigerbotics7125.robot.subsystem.Vision;
import io.github.tigerbotics7125.tigerlib.input.controller.XboxController;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger.ActivationCondition;
import io.github.tigerbotics7125.tigerlib.util.JoystickUtil;

public class RobotContainer {

    XboxController mDriver = new XboxController(DRIVER_CONTROLLER_PORT);
    XboxController mOperator = new XboxController(OPERATOR_CONTROLLER_PORT);
    Trigger mRioUserButton = new Trigger(RobotController::getUserButton);

    Vision mVision = new Vision();
    Drivetrain mDrivetrain = new Drivetrain();
    Intake mIntake = new Intake();
    SuperStructure mSuperStruc = new SuperStructure();
    Compressor mCompressor = new Compressor(PNEUMATICS_MODULE_TYPE);

    AutoPilot mAutoPilot = new AutoPilot(mDrivetrain, OperatorInterface::getAutoPilotPoint);

    Field2d mField = new Field2d();

    SendableChooser<Auto> mAutoChooser = new SendableChooser<>();

    public RobotContainer() {
        mCompressor.enableDigital();
        configTriggers();
        configDefaultCommands();
        configDashboard();

        mDrivetrain.setFieldOriented(false);
        mDrivetrain.setTurningMode(TurningMode.JOYSTICK_DIRECT);
    }

    public Command getAutoCommand() {

        Auto auto = mAutoChooser.getSelected();

        MecanumAutoBuilder autoBuilder =
                new MecanumAutoBuilder(
                        mDrivetrain::getPose,
                        mDrivetrain::setPose,
                        new PIDConstants(1, 0, 0),
                        new PIDConstants(1, 0, 0),
                        mDrivetrain::setChassisSpeeds,
                        auto.getEventMap(),
                        true,
                        mDrivetrain);

        return autoBuilder.fullAuto(auto.getPath());
    }

    private void configTriggers() {
        // Driver
        mDriver.b().trigger(mDrivetrain::resetGyro);
        mDriver.lb().trigger(() -> mDrivetrain.setTurningMode(TurningMode.JOYSTICK_DIRECT));
        mDriver.rb().trigger(() -> mDrivetrain.setTurningMode(TurningMode.JOYSTICK_ANGLE));
        mDriver.y().trigger(() -> mDrivetrain.setTurningMode(TurningMode.HEADING_LOCK));

        mDriver.start()
                .activate(ActivationCondition.WHILE_HIGH)
                .trigger(mAutoPilot.getAutoPilotCommand());

        // Operator
        mOperator.pov.left().trigger(OperatorInterface.selectLeft());
        mOperator.pov.up().trigger(OperatorInterface.selectUp());
        mOperator.pov.right().trigger(OperatorInterface.selectRight());
        mOperator.pov.down().trigger(OperatorInterface.selectDown());

        mOperator.lb().trigger(OperatorInterface.selectLeftSubstation());
        mOperator.rb().trigger(OperatorInterface.selectRightSubstation());

        mOperator.start().trigger(OperatorInterface.toggleZone());

        mOperator.a().trigger(mIntake.releaseObject());
        mOperator.b().trigger(mIntake.grabObject());
        mOperator.y().trigger(mIntake::intake);
        mOperator.x().trigger(mIntake::disable);

        mOperator
                .rb()
                .activate(WHILE_HIGH)
                .trigger(
                        Commands.runOnce(
                                () ->
                                        mSuperStruc.armDuty(
                                                JoystickUtil.clamp(
                                                        mOperator.rightY().get(), -.2, .2))));
        mOperator
                .lb()
                .activate(WHILE_HIGH)
                .trigger(
                        Commands.runOnce(
                                () ->
                                        mSuperStruc.wristDuty(
                                                JoystickUtil.clamp(
                                                        mOperator.rightY().get(), -.2, .2))));
        /*
        mOperator.pov.up().activate(WHILE_HIGH).trigger(Commands.runOnce(() -> mSuperStruc.elevatorDuty(.5)));
        mOperator.pov.down().activate(WHILE_HIGH).trigger(Commands.runOnce(() -> mSuperStruc.elevatorDuty(-0.5)));
        */

        mOperator.pov.up().activate(WHILE_HIGH).trigger(mSuperStruc.elevatorUp());
        mOperator.pov.up().activate(WHILE_HIGH).trigger(mSuperStruc.elevatorDown());
    }

    public void configDashboard() {
        Shuffleboard.getTab("odometry").add(mField);

        mAutoChooser.setDefaultOption("NONE", new None());
        mAutoChooser.addOption("Center Delay to Wall", new CenterDelayMobility());
    }

    private void configDefaultCommands() {
        mDrivetrain.setDefaultCommand(
                mDrivetrain.driveCmd(
                        mDriver.leftY()::get,
                        mDriver.leftX()::get,
                        mDriver.rightY()::get,
                        mDriver.rightX()::get,
                        true));

        mIntake.setDefaultCommand(Commands.run(mIntake::disable, mIntake));
        mSuperStruc.setDefaultCommand(Commands.run(mSuperStruc::disable, mSuperStruc));
    }

    /** Periodic call, always runs. */
    public void periodic() {

        // mVision.getEstimatedPose().ifPresent(mDrivetrain::addVisionEstimate);

        mField.setRobotPose(mDrivetrain.getPose());
        mField.getObject("Flipped").setPose(AllianceFlipUtil.apply(mDrivetrain.getPose()));
        mField.getObject("Community")
                .setPoses(AutoPilotConstants.AutoPilotBoundary.COMMUNITY.getPoses());
        mField.getObject("LoadingZone")
                .setPoses(AutoPilotConstants.AutoPilotBoundary.LOADING_ZONE.getPoses());
    }

    /** Periodic call, only runs during simulation. */
    public void simulationPeriodic() {
        // update sim vision with our robot pose.
        mVision.update(mDrivetrain.getPose());
    }
}
