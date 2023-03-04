/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.sysid.logging;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SysIdDrivetrainLogger extends SysIdLogger {

    double primaryMotorVoltage = 0.0;
    double secondaryMotorVoltage = 0.0;

    public double getLeftMotorVoltage() {
        return primaryMotorVoltage;
    }

    public double getRightMotorVoltage() {
        return secondaryMotorVoltage;
    }

    public void log(
            double leftVoltage,
            double rightVoltage,
            double leftPosition,
            double rightPosition,
            double leftVelocity,
            double rightVelocity,
            double measuredAngle,
            double angularRate) {
        updateData();
        SmartDashboard.putNumber("gyroAngle", measuredAngle);
        if (data.size() < dataVectorSize) {
            double[] dataPacket =
                    new double[] {
                        timestamp,
                        leftVoltage,
                        rightVoltage,
                        leftPosition,
                        rightPosition,
                        leftVelocity,
                        rightVelocity,
                        measuredAngle,
                        angularRate
                    };
            for (double d : dataPacket) {
                data.add(d);
            }
        }
        primaryMotorVoltage = rotate ? -1 * motorVoltage : 1 * motorVoltage;
        secondaryMotorVoltage = motorVoltage;
        SmartDashboard.putNumber("motorVoltage", motorVoltage);
        SmartDashboard.putBoolean("wrongmech", isWrongMechanism());
    }

    public void reset() {
        super.reset();
        primaryMotorVoltage = 0.0;
        secondaryMotorVoltage = 0.0;
    }

    boolean isWrongMechanism() {
        return !mechanism.equals("Drivetrain") && !mechanism.equals("Drivetrain (Angular)");
    }
}
