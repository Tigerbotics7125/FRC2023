/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import static io.github.tigerbotics7125.robot.constants.OIConstants.*;
import static io.github.tigerbotics7125.tigerlib.input.trigger.Trigger.ActivationCondition.*;

import com.pathplanner.lib.auto.MecanumAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import io.github.tigerbotics7125.robot.auto.Auto;
import io.github.tigerbotics7125.robot.auto.CenterDelayMobility;
import io.github.tigerbotics7125.robot.auto.None;
import io.github.tigerbotics7125.robot.constants.RobotConstants;
import io.github.tigerbotics7125.robot.subsystem.Drivetrain;
import io.github.tigerbotics7125.robot.subsystem.Drivetrain.TurningMode;
import io.github.tigerbotics7125.robot.subsystem.Intake;
import io.github.tigerbotics7125.robot.subsystem.SuperStructure;
import io.github.tigerbotics7125.robot.subsystem.Vision;
import io.github.tigerbotics7125.tigerlib.CommandRobot;
import io.github.tigerbotics7125.tigerlib.input.controller.XboxController;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger;
import org.photonvision.PhotonCamera;

public class Robot extends CommandRobot {

    // Gamepads & Buttons
    public static XboxController mDriver;
    public static XboxController mOperator;
    public static Trigger mRioUserButton;

    // Subsystems
    public static Vision mVision;
    public static Drivetrain mDrivetrain;
    public static Intake mIntake;
    public static SuperStructure mSuperStruc;

    // Components
    public static PowerDistribution mPDH = new PowerDistribution();
    public static Compressor mCompressor = new Compressor(RobotConstants.PNEUMATICS_MODULE_TYPE);

    // Commands
    public static AutoPilot mAutoPilot;

    // Chooser
    public static SendableChooser<Auto> mAutoChooser;

    @Override
    public void robotInit() {
        CANSparkMax.enableExternalUSBControl(true);
        PhotonCamera.setVersionCheckEnabled(false);
        DriverStation.silenceJoystickConnectionWarning(true);

        String initStr = "Initialized: %s.\n";

        mDriver = new XboxController(DRIVER_CONTROLLER_PORT);
        mOperator = new XboxController(OPERATOR_CONTROLLER_PORT);
        mRioUserButton = new Trigger(RobotController::getUserButton);
        System.out.printf(initStr, "Gamepads & Buttons");

        mVision = new Vision();
        System.out.printf(initStr, "Vision");
        mDrivetrain = new Drivetrain();
        System.out.printf(initStr, "Drivetrain");
        mIntake = new Intake();
        System.out.printf(initStr, "Intake");
        mSuperStruc = new SuperStructure();
        System.out.printf(initStr, "Super Structure");

        mCompressor.enableDigital();
        System.out.printf(initStr, "Compressor");

        mAutoPilot = new AutoPilot(mDrivetrain, OperatorInterface::getAutoPilotPoint);
        System.out.printf(initStr, "AutoPilot");

        mAutoChooser = new SendableChooser<>();
        mAutoChooser.setDefaultOption("NONE", new None());
        mAutoChooser.addOption("Center Delay to Wall", new CenterDelayMobility());
        System.out.printf(initStr, "Auto Chooser");

        configTriggers();
        System.out.printf(initStr, "Triggers");

        configDefaultCommands();
        System.out.printf(initStr, "Default Commands");

        OperatorInterface.init();
        System.out.printf(initStr, "Operator Interface");

        // TODO: use Constants file to set these
        mDrivetrain.setFieldOriented(false);
        mDrivetrain.setTurningMode(TurningMode.JOYSTICK_DIRECT);
    }

    @Override
    public void robotPeriodic() {
        OperatorInterface.update();
        Shuffleboard.update();
        SmartDashboard.updateValues();
    }

    @Override
    public void autonomousInit() {
        OperatorInterface.startMatchSegment();

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

        Command autoCmd = autoBuilder.fullAuto(auto.getPath());
        CommandScheduler.getInstance().schedule(autoCmd);
    }

    @Override
    public void teleopInit() {
        OperatorInterface.startMatchSegment();
    }

    @Override
    public void simulationInit() {
        // Disable joystick warnings in sim.
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().run();
        mVision.update(mDrivetrain.getPose());
    }

    private void configTriggers() {
        // Driver
        mDriver.b().trigger(ON_RISING, Commands.runOnce(mDrivetrain::resetGyro));
        mDriver.lb()
                .trigger(
                        ON_RISING,
                        Commands.runOnce(
                                () -> mDrivetrain.setTurningMode(TurningMode.JOYSTICK_DIRECT)));
        mDriver.rb()
                .trigger(
                        ON_RISING,
                        Commands.runOnce(
                                () -> mDrivetrain.setTurningMode(TurningMode.JOYSTICK_ANGLE)));
        mDriver.y()
                .trigger(
                        ON_RISING,
                        Commands.runOnce(
                                () -> mDrivetrain.setTurningMode(TurningMode.HEADING_LOCK)));

        mDriver.start().trigger(WHILE_HIGH, mAutoPilot.getAutoPilotCommand());

        // Operator
        mOperator.pov.left().trigger(ON_RISING, OperatorInterface.selectLeft());
        mOperator.pov.up().trigger(ON_RISING, OperatorInterface.selectUp());
        mOperator.pov.right().trigger(ON_RISING, OperatorInterface.selectRight());
        mOperator.pov.down().trigger(ON_RISING, OperatorInterface.selectDown());

        mOperator.lb().trigger(ON_RISING, OperatorInterface.selectLeftSubstation());
        mOperator.rb().trigger(ON_RISING, OperatorInterface.selectRightSubstation());

        mOperator.start().trigger(ON_RISING, OperatorInterface.toggleZone());

        mOperator.a().trigger(WHILE_HIGH, mIntake.intakeOut());
        mOperator.y().trigger(WHILE_HIGH, mIntake.intakeAuto());
        mOperator.b().trigger(ON_RISING, mIntake.grippersOpen());
        mOperator.x().trigger(ON_RISING, mIntake.grippersClose());

        mOperator.pov.up().trigger(WHILE_HIGH, mSuperStruc.elevatorUp());
        mOperator.pov.down().trigger(WHILE_HIGH, mSuperStruc.elevatorDown());
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
}
