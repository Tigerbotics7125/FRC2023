/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.subsystem;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.tigerbotics7125.robot.Robot;
import io.github.tigerbotics7125.robot.constants.DashboardConstants;

public class SuperStructure extends SubsystemBase {

    public enum State {
        DISABLE(
                Commands.runOnce(
                        () -> {
                            Robot.mElev.disable();
                            Robot.mArm.disable();
                            Robot.mWrist.disable();
                        })),

        HOME(Elevator.State.HOME, Arm.State.HOME, Wrist.State.HOME),
        STOW(Elevator.State.HOME, Arm.State.HOME, Wrist.State.PARALLEL_ARM),

        GROUND_INTAKE(
                Elevator.State.GROUND_INTAKE, Arm.State.GROUND_INTAKE, Wrist.State.GROUND_INTAKE),
        SUBSTATION(Elevator.State.THREE_QUARTERS, Arm.State.UP, Wrist.State.PARALLEL_ARM),

        HIGH_CUBE(Elevator.State.HIGH_CUBE_SCORE, Arm.State.HIGH_CUBE, Wrist.State.PARALLEL_ARM),
        HIGH_CONE(),
        MID_CUBE(),
        MID_CONE(),
        HYBRID(),
        ;

        public final Elevator.State mElevState;
        public final Arm.State mArmState;
        public final Wrist.State mWristState;
        public final CommandBase mRoutine;

        private State(Elevator.State elev, Arm.State arm, Wrist.State wrist) {
            mElevState = elev;
            mArmState = arm;
            mWristState = wrist;
            mRoutine = null;
        }

        private State(CommandBase routine) {
            mElevState = null;
            mArmState = null;
            mWristState = null;
            mRoutine = routine;
        }

        private State() {
            this(Commands.none());
        }
    }

    private final Elevator mElev;
    private final Arm mArm;
    private final Wrist mWrist;

    private State mState;

    private double mSimArmOffset = 15D;
    private double mSimWristOffset = 90D;
    private Mechanism2d mMech;
    private MechanismRoot2d mRoot;
    private MechanismLigament2d mElevMech;
    private MechanismLigament2d mArmMech;
    private MechanismLigament2d mWristMech;

    public SuperStructure(Elevator elev, Arm arm, Wrist wrist) {
        mElev = elev;
        mArm = arm;
        mWrist = wrist;

        var tab = DashboardConstants.SUPER_STRUC_TAB;
        tab.addDouble("Elev Setpoint", () -> mElev.getController().getGoal().position);
        tab.addDouble("Elev Pos", () -> mElev.getMeasurement());
        tab.addDouble("Arm Setpoint", () -> mArm.getController().getGoal().position);
        tab.addDouble("Arm Pos", () -> mArm.getMeasurement());
        tab.addDouble("Wrist Setpoint", () -> mWrist.getController().getGoal().position);
        tab.addDouble("Wrist Pos", () -> mWrist.getMeasurement());

        mMech = new Mechanism2d(2, 2);
        mRoot = mMech.getRoot("Robot", 0, 0);
        mElevMech =
                mRoot.append(
                        new MechanismLigament2d("Elev", 0, 45, 5, new Color8Bit(Color.kAliceBlue)));
        mArmMech =
                mElevMech.append(
                        new MechanismLigament2d(
                                "Arm",
                                Units.inchesToMeters(18),
                                0,
                                5,
                                new Color8Bit(Color.kAntiqueWhite)));
        mWristMech =
                mArmMech.append(
                        new MechanismLigament2d(
                                "Wrist",
                                Units.inchesToMeters(12),
                                0,
                                5,
                                new Color8Bit(Color.kAqua)));

        if (Robot.isSimulation()) tab.add("mech", mMech);
    }

    // TODO: requirements
    public CommandBase setState(State state) {
        mState = state;
        if (state.mRoutine != null) return state.mRoutine;
        else
            return Commands.parallel(
                    mElev.setState(state.mElevState),
                    mArm.setState(state.mArmState),
                    mWrist.setState(state.mWristState));
    }

    @Override
    public void simulationPeriodic() {
        mElevMech.setLength(mElev.getMeasurement());
        mArmMech.setAngle(-Units.radiansToDegrees(mArm.getMeasurement()) - mSimArmOffset);
        mWristMech.setAngle(mWrist.getMeasurement() - mSimWristOffset);
    }
}
