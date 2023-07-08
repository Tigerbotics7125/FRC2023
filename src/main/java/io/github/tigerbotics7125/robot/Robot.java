/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import static io.github.tigerbotics7125.tigerlib.input.trigger.Trigger.ActivationCondition.ON_RISING;

import com.pathplanner.lib.PathPlannerTrajectory;
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
import io.github.tigerbotics7125.robot.auto.*;
import io.github.tigerbotics7125.robot.commands.SuperStructure;
import io.github.tigerbotics7125.robot.constants.ElevatorConstants;
import io.github.tigerbotics7125.robot.subsystem.*;
import io.github.tigerbotics7125.tigerlib.CommandRobot;
import java.util.List;

public class Robot extends CommandRobot {

    // * Components
    // Operator Interface, contains controller objects and methods.
    public static OI mOI;
    // PH Object, used to create and control pneumatic devices.
    public static PneumaticHub mPH;

    // * Subsystems
    // Controls the wheels and manages odometry.
    public static Drivetrain mDrivetrain;
    // Vision has still not actually been used during competition which is omega sadge.
    // Controls the cameras, and feeds data to the drivetrain, or more approprietly, odometry.
    // public static Vision mVision;
    // Controls the elevator.
    public static Elevator2 mElev;
    // Controls the arm.
    public static Arm2 mArm;
    // Controls the wrist.
    public static Wrist2 mWrist;
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
        SmartDashboard.putBoolean("codeReady", false);
        // literally stfu
        DriverStation.silenceJoystickConnectionWarning(true);

        mOI = new OI();

        // Turn the compressor on using the digital pressure sensor.
        mPH = new PneumaticHub();
        mPH.enableCompressorDigital();

        mDrivetrain = new Drivetrain();
        // mVision = new Vision();
        mElev = new Elevator2();
        mArm = new Arm2();
        mWrist = new Wrist2();
        // Obviously the SS requires the subsystems to control them.
        // mSuperStructure = new SuperStructure(mElev, mArm, mWrist);
        mIntake = new Intake();

        // The constructor uses a "Supplier" which in this case is a method reference / lambda, you
        // can look up what those are.
        mAutoPilot = new AutoPilot(mDrivetrain, Dashboard::getAutoPilotPoint);

        mAutoChooser = new SendableChooser<>();
        // Add options to the chooser, and by default do nothing.
        mAutoChooser.setDefaultOption("NONE", new None());
        // mAutoChooser.addOption("Center Delay to Wall", new CenterDelayMobility());
        // mAutoChooser.addOption("Cable Run", new CableRun());
        mAutoChooser.addOption("Standstill Cube", new StandstillCube(mElev, mArm, mWrist, mIntake));
        // mAutoChooser.addOption("StandstillCube", new Auto() );

        // Configure button bindings.
        configTriggers();
        // Configure what subsystems do when not running a command.
        configDefaultCommands();

        // Setup the dashboard programatically so its always the same.
        Dashboard.init();

        // Allow SparkMaxs to be controlled from REV Hardware Client while on rio CAN bus.
        // Supposed to fix that, but its kinda jank and still doesn't work all the time.
        CANSparkMax.enableExternalUSBControl(true);

        SmartDashboard.putBoolean("codeReady", true);
        System.out.println();
        System.out.println();
        System.out.println();
        System.out.println("** ROBOT CODE INITIALIZED **");
        System.out.println();
        System.out.println();
        System.out.println();
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
        List<PathPlannerTrajectory> path = auto.getPath();
        if (path.isEmpty()) path = List.of(new PathPlannerTrajectory());

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
        Command autoCmd =
                auto.preCommand().andThen(autoBuilder.fullAuto(path)).andThen(auto.postCommand());
        CommandScheduler.getInstance().schedule(autoCmd);
    }

    @Override
    public void teleopInit() {
        Dashboard.startMatchSegment();
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

        // mOI.mOp.y().trigger(ON_RISING, SuperStructure.ensureSafe(mArm, mWrist, mIntake));
        // mOI.mOp.pov.up().trigger(ON_RISING, mElev.setState(Elevator.ElevState.HALF));
        // mOI.mOp.pov.down().trigger(ON_RISING, mElev.setState(Elevator.ElevState.HOME));

        mOI.mOp.lb().trigger(ON_RISING, mIntake.cubeIntakeRoutine(mOI.rumble(Controller.BOTH)));
        mOI.mOp.rb().trigger(ON_RISING, mIntake.outakeRoutine());

        mOI.mOp.a().trigger(ON_RISING, SuperStructure.home(mElev, mArm, mWrist, mIntake));
        mOI.mOp
                .pov
                .down()
                .trigger(ON_RISING, SuperStructure.groundIntake(mElev, mArm, mWrist, mIntake));
        mOI.mOp
                .pov
                .right()
                .trigger(ON_RISING, SuperStructure.midCube(mElev, mArm, mWrist, mIntake));

        mOI.mOp.pov.up().trigger(ON_RISING, SuperStructure.highCube(mElev, mArm, mWrist, mIntake));

        // mOI.mOp.y().trigger(ON_RISING, mElev.setState(ElevState.QUARTER));
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
    }
}
