/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import static io.github.tigerbotics7125.tigerlib.input.trigger.Trigger.ActivationCondition.*;

import com.pathplanner.lib.auto.MecanumAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import io.github.tigerbotics7125.robot.OI.Controller;
import io.github.tigerbotics7125.robot.auto.Auto;
import io.github.tigerbotics7125.robot.auto.CableRun;
import io.github.tigerbotics7125.robot.auto.CenterDelayMobility;
import io.github.tigerbotics7125.robot.auto.None;
import io.github.tigerbotics7125.robot.constants.ElevatorConstants;
import io.github.tigerbotics7125.robot.subsystem.*;
import io.github.tigerbotics7125.robot.subsystem.Drivetrain.TurningMode;
import io.github.tigerbotics7125.tigerlib.CommandRobot;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger;

public class Robot extends CommandRobot {

    // Components
    public static OI mOI;
    public static PowerDistribution mPDH;
    public static PneumaticHub mPH;
    public static Compressor mCompressor;

    // Subsystems
    public static Drivetrain mDrivetrain;
    public static Vision mVision;
    public static Elevator mElev;
    public static Arm mArm;
    public static Wrist mWrist;
    public static SuperStructure mSuperStructure;
    public static Intake mIntake;

    // Commands
    public static AutoPilot mAutoPilot;

    // Chooser
    public static SendableChooser<Auto> mAutoChooser;

    @Override
    public void robotInit() {

        mOI = new OI();
        mPDH = new PowerDistribution();
        mPH = new PneumaticHub();
        mPH.enableCompressorDigital();

        mDrivetrain = new Drivetrain();
        mVision = new Vision();
        mElev = new Elevator();
        mArm = new Arm();
        mWrist = new Wrist();
        mSuperStructure = new SuperStructure(mElev, mArm, mWrist);
        mIntake = new Intake();

        mAutoPilot = new AutoPilot(mDrivetrain, Dashboard::getAutoPilotPoint);

        mAutoChooser = new SendableChooser<>();
        mAutoChooser.setDefaultOption("NONE", new None());
        mAutoChooser.addOption("Center Delay to Wall", new CenterDelayMobility());
        mAutoChooser.addOption("Cable Run", new CableRun());

        configTriggers();

        configDefaultCommands();

        Dashboard.init();

        // Allow SparkMaxs to be controlled from REV Hardware Client while on rio CAN bus.
        CANSparkMax.enableExternalUSBControl(true);
        // TODO: Use constants file to set these
        mDrivetrain.setFieldOriented(true);
        mDrivetrain.setTurningMode(TurningMode.JOYSTICK_DIRECT);
    }

    @Override
    public void robotPeriodic() {
        Dashboard.update();
        Shuffleboard.update();
        SmartDashboard.updateValues();
    }

    @Override
    public void autonomousInit() {
        Dashboard.startMatchSegment();

        Auto auto = mAutoChooser.getSelected();

        // IndexOutOfBoundsException will occur with paths that are empty, so escape here.
        if (auto.getPath().isEmpty()) return;

        MecanumAutoBuilder autoBuilder =
                new MecanumAutoBuilder(
                        mDrivetrain::getPose,
                        mDrivetrain::resetOdometry,
                        new PIDConstants(1, 0, 0),
                        new PIDConstants(1, 0, 0),
                        mDrivetrain::setChassisSpeeds,
                        auto.getEventMap(),
                        false,
                        mDrivetrain);

        Command autoCmd = autoBuilder.fullAuto(auto.getPath());
        CommandScheduler.getInstance().schedule(autoCmd);
    }

    @Override
    public void teleopInit() {
        Dashboard.startMatchSegment();
    }

    @Override
    public void simulationInit() {
        // Disable joystick warnings in sim.
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().run();
    }

    private void configTriggers() {
        // ^ Driver
        // * Drive trainslation via left joystick
        // * Drive rotation via right joystick
        // * Alter behavior
        mOI.mDriver.b().trigger(ON_RISING, Commands.runOnce(mDrivetrain::resetGyro));
        mOI.mDriver.x().trigger(ON_RISING, mDrivetrain.directTurning());
        // mOI.mDriver.rb().trigger(ON_RISING, mDrivetrain.angleTurning());
        mOI.mDriver.y().trigger(ON_RISING, mDrivetrain.lockTurning());
        /*
        // * Auto align and score (eventually)
        mOI.mDriver.start().trigger(WHILE_HIGH, mAutoPilot.getAutoPilotCommand());
        */

        // ^ Operator
        // * Node / Substation Selector
        mOI.mOp.pov.left().trigger(ON_RISING, Dashboard.selectLeft());
        mOI.mOp.pov.up().trigger(ON_RISING, Dashboard.selectUp());
        mOI.mOp.pov.right().trigger(ON_RISING, Dashboard.selectRight());
        mOI.mOp.pov.down().trigger(ON_RISING, Dashboard.selectDown());
        // ~ Toggle slection between nodes & substations
        mOI.mOp.start().trigger(ON_RISING, Dashboard.toggleZone());
        // * Intake Control
        mOI.mOp.lb().trigger(ON_RISING, mIntake.cubeIntakeRoutine(mOI.rumble(Controller.BOTH)));
        mOI.mOp.rb().trigger(ON_RISING, mIntake.coneIntakeRoutine());
        new Trigger(() -> mOI.mOp.rt().get() > .5).trigger(WHILE_HIGH, mIntake.outakeRoutine());
        // * Super Structure Control
        mOI.mOp.a().trigger(ON_RISING, mSuperStructure.setState(Dashboard.getSuperStrucState()));
        mOI.mOp.b().trigger(ON_RISING, mSuperStructure.setState(SuperStructure.State.STOW));
        mOI.mOp
                .x()
                .trigger(ON_RISING, mSuperStructure.setState(SuperStructure.State.GROUND_INTAKE));
        // mOI.mOp.y().trigger(ON_RISING, mSuperStructure.setState(SuperStructure.State.HIGH_CUBE));
    }

    /** Sets all subsystems default commands. */
    private void configDefaultCommands() {
        mDrivetrain.setDefaultCommand(
                mDrivetrain.driveWithLimits(
                        mOI.mDriver.leftY()::get,
                        () -> mOI.mDriver.leftX().get() * -1,
                        mOI.mDriver.rightY()::get,
                        () -> mOI.mDriver.rightX().get() * -1,
                        () -> 1 - (mElev.getMeasurement() / ElevatorConstants.MAX_HEIGHT)));
        mIntake.setDefaultCommand(mIntake.disable());
    }
}
