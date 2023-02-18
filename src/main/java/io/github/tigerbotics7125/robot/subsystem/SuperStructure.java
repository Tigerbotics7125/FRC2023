/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.tigerbotics7125.lib.MB1122;
import io.github.tigerbotics7125.robot.Robot;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger.ActivationCondition;

public class SuperStructure extends SubsystemBase {

    private enum BrakeState {
        BRAKING,
        COASTING
    }

    private BrakeState mArmBrakeState;

    private MB1122 mElevatorDistanceSensor;
    private CANSparkMax mLeftElevator;
    private CANSparkMax mRightElevator;

    private CANSparkMax mArm;
    private DoubleSolenoid mArmBrake;

    private double tubeThick = 0.0254;
    private double elevX, elevY, elevHeight;
    private double elevMin, elevMax;
    private Mechanism2d mMechanism;
    private MechanismRoot2d stage0Root, stage1Root, stage2Root, armRoot;
    private MechanismLigament2d stage0, stage1, stage2, arm, intake;

    private SingleJointedArmSim mArmSim;

    public SuperStructure() {
        mElevatorDistanceSensor = new MB1122(new AnalogInput(0));
        mLeftElevator = new CANSparkMax(11, MotorType.kBrushless);
        mRightElevator = new CANSparkMax(12, MotorType.kBrushless);
        mArm = new CANSparkMax(13, MotorType.kBrushless);
        mArmBrake = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

        mRightElevator.follow(mLeftElevator);

        // Trigger to control braking of the motor and disk brake at the same time.
        new Trigger(this::isArmBraking)
                .activate(ActivationCondition.ON_RISING)
                .trigger(() -> this.brakeArm())
                .not()
                .trigger(() -> this.coastArm());

        ShuffleboardTab tab = Shuffleboard.getTab("SuperStruc");
        tab.addBoolean("armBraking", this::isArmBraking);

        if (Robot.isSimulation()) {

            elevMax = 1.334530;
            elevMin = 0;

            elevX = 1;
            elevY = 0;
            elevHeight = elevMin;

            double stage0Meters = 0.685800;
            double stage1Meters = 0.711200;
            double stage2Meters = 0.177800;
            double armMeters = 1.33;
            double intakeMeters = .2;

            mMechanism = new Mechanism2d(3, 3);

            stage0Root = mMechanism.getRoot("stage0Root", 0, 0);
            stage0 =
                    stage0Root.append(
                            new MechanismLigament2d(
                                    "stage0",
                                    stage0Meters,
                                    90,
                                    15,
                                    new Color8Bit(Color.kBlueViolet)));
            stage1Root = mMechanism.getRoot("stage1Root", 0, 0);
            stage1 =
                    stage1Root.append(
                            new MechanismLigament2d(
                                    "stage1",
                                    stage1Meters,
                                    90,
                                    10,
                                    new Color8Bit(Color.kLightBlue)));
            stage2Root = mMechanism.getRoot("stage2Root", 0, 0);
            stage2 =
                    stage2Root.append(
                            new MechanismLigament2d(
                                    "stage2",
                                    stage2Meters,
                                    90,
                                    5,
                                    new Color8Bit(Color.kFirstBlue)));
            armRoot = mMechanism.getRoot("armRoot", 0, 0);
            arm =
                    armRoot.append(
                            new MechanismLigament2d(
                                    "arm", armMeters, 180, 10, new Color8Bit(Color.kIndianRed)));
            intake =
                    arm.append(
                            new MechanismLigament2d(
                                    "intake", intakeMeters, 0, 5, new Color8Bit(Color.kFirstRed)));
        }
        tab.add("Mech", mMechanism);

        mArmSim =
                new SingleJointedArmSim(
                        DCMotor.getNEO(2),
                        20.0,
                        2.84575616194,
                        1.1938,
                        Math.PI / 2.0,
                        3.0 * Math.PI / 2.0,
                        true);
    }

    @Override
    public void simulationPeriodic() {
        stage0Root.setPosition(elevX, elevY);
        double stage1Height = elevY + (elevHeight / 2.0);
        stage1Root.setPosition(elevX, stage1Height);
        double elevPercentage = elevHeight / elevMax;
        double stage2Height =
                stage1Height
                        + (elevPercentage * ((0.711200 - 2.0 * tubeThick) - 0.177800))
                        + tubeThick;
        stage2Root.setPosition(elevX, stage2Height);
        armRoot.setPosition(elevX, stage2Height + 0.177800 / 2.0);
        arm.setAngle(Units.radiansToDegrees(mArmSim.getAngleRads()));
    }

    public void moveElevator(double horizontal, double vertical) {
        elevHeight += vertical;
        if (elevHeight > elevMax) elevHeight = elevMax;
        if (elevHeight < elevMin) elevHeight = elevMin;

        // arm.setAngle(arm.getAngle() + horizontal);
        intake.setAngle(180 - arm.getAngle());

        mArmSim.setInputVoltage(horizontal * 12.0);
        mArmSim.update(.02);
    }

    /** @return If the arm is braking. */
    public boolean isArmBraking() {
        return mArmBrakeState == BrakeState.BRAKING;
    }

    /** Enable all braking devices for the arm. */
    public void brakeArm() {
        mArm.setIdleMode(IdleMode.kBrake);
        mArmBrake.set(Value.kForward);
    }

    /** Disable all braking devices for the arm. */
    public void coastArm() {
        mArm.setIdleMode(IdleMode.kCoast);
        mArmBrake.set(Value.kReverse);
    }
}
