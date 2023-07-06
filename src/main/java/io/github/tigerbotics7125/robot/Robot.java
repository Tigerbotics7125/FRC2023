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
import io.github.tigerbotics7125.tigerlib.CommandRobot;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger;

public class Robot extends CommandRobot {

    // * Components
    // Operator Interface, contains controller objects and methods.
    public static OI mOI;
    // PDH Object, can (and should...) be used to track and manage power usage.
    public static PowerDistribution mPDH;
    // PH Object, used to create and control pneumatic devices.
    public static PneumaticHub mPH;

    // * Subsystems
    // Controls the wheels and manages odometry.
    public static Drivetrain mDrivetrain;
    // Controls the cameras, and feeds data to the drivetrain, or more approprietly, odometry.
    public static Vision mVision;
    // Controls the elevator.
    public static Elevator mElev;
    // Controls the arm.
    public static Arm mArm;
    // Controls the wrist.
    public static Wrist mWrist;
    // Controls the elevator, arm, and wrist together at the same time.
    public static SuperStructure mSuperStructure;
    // Controls the intake.
    public static Intake mIntake;

    // * Commands
    // Allows the robot to auto align to nodes.
    public static AutoPilot mAutoPilot;

    // * Misc.
    // Lets the user pick the auto that runs.
    public static SendableChooser<Auto> mAutoChooser;

    /** Initializes all of the objects above plus sets up a couple things. */
    @Override
    public void robotInit() {

        mOI = new OI();
        mPDH = new PowerDistribution();
        mPH = new PneumaticHub();
        // Turn the compressor on using the digital pressure sensor.
        mPH.enableCompressorDigital();

        mDrivetrain = new Drivetrain();
        mVision = new Vision();
        mElev = new Elevator();
        mArm = new Arm();
        mWrist = new Wrist();
        // Obviously the SS requires the subsystems to control them.
        mSuperStructure = new SuperStructure(mElev, mArm, mWrist);
        mIntake = new Intake();

        // The constructor uses a "Supplier" which in this case is a method reference / lambda, you
        // can look up what those are.
        mAutoPilot = new AutoPilot(mDrivetrain, Dashboard::getAutoPilotPoint);

        mAutoChooser = new SendableChooser<>();
        // Add options to the chooser, and by default do nothing.
        mAutoChooser.setDefaultOption("NONE", new None());
        mAutoChooser.addOption("Center Delay to Wall", new CenterDelayMobility());
        mAutoChooser.addOption("Cable Run", new CableRun());

        // Configure button bindings.
        configTriggers();
        // Configure what subsystems do when not running a command.
        configDefaultCommands();

        // Setup the dashboard programatically so its always the same.
        Dashboard.init();

        // Allow SparkMaxs to be controlled from REV Hardware Client while on rio CAN bus.
        // Supposed to fix that, but its kinda jank and still doesn't work all the time.
        CANSparkMax.enableExternalUSBControl(true);
    }

    /** If you don't know what periodic means look it up. */
    @Override
    public void robotPeriodic() {
        Dashboard.update();
        Shuffleboard.update();
        SmartDashboard.updateValues();
    }

    @Override
    public void autonomousInit() {
        // This call restarts the time on the dashboard so we can have an estimated time without
        // having to look at the match clock.
        Dashboard.startMatchSegment();

        // Gets the autonomous routine from the chooser.
        Auto auto = mAutoChooser.getSelected();

        // Gets the path created by path planner, and check that its not empty (causes IOoB error)
        if (auto.getPath().isEmpty()) return;

        // The auto builder - it creates the path following command that is run during auto.
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

        // Create the command and schedule it.
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
        // Updates SparkMax values during sim, its kinda jank cuz rev didn't do such a great job,
        // just hope it's better for next year.
        REVPhysicsSim.getInstance().run();
    }

    private void configTriggers() {
        // ^ Driver
        // * Drive trainslation via left joystick
        // * Drive rotation via right joystick
        // * Alter behavior
        mOI.mDriver.b().trigger(ON_RISING, Commands.runOnce(mDrivetrain::resetGyro));
        // These are commented out to prevent the driver accidentely changing and not knowing how to
        // get back.
        // mOI.mDriver.x().trigger(ON_RISING, mDrivetrain.directTurning());
        // mOI.mDriver.rb().trigger(ON_RISING, mDrivetrain.angleTurning());
        // mOI.mDriver.y().trigger(ON_RISING, mDrivetrain.lockTurning());
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

    /**
     * Sets all subsystems default commands. In other words, what they should be doing when not
     * explicetly running a different command.
     */
    private void configDefaultCommands() {
        mDrivetrain.setDefaultCommand(
                mDrivetrain.driveWithLimits(
                        mOI.mDriver.leftY()::get,
                        () -> mOI.mDriver.leftX().get() * -1,
                        mOI.mDriver.rightY()::get,
                        () -> mOI.mDriver.rightX().get() * -1,
                        () -> 1 - (mElev.getMeasurement() / ElevatorConstants.MAX_HEIGHT)));
        mIntake.setDefaultCommand(mIntake.disable());
        mArm.setDefaultCommand(mArm.manualDrive(mOI.mOp.leftY()::get));
        mWrist.setDefaultCommand(mWrist.manualDrive(mOI.mOp.rightY()::get));
    }
}
