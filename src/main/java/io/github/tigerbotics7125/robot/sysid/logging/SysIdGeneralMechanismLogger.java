/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.sysid.logging;

/** Add your docs here. */
public class SysIdGeneralMechanismLogger extends SysIdLogger {
    private double primaryMotorVoltage = 0.0;

    public double getMotorVoltage() {
        return primaryMotorVoltage;
    }

    public void log(double voltage, double measuredPosition, double measuredVelocity) {
        // set desired voltage
        updateData();
        // if we have room left in the data list
        if (data.size() < dataVectorSize) {
            // add datapoints to list
            double[] dataPacket =
                    new double[] {timestamp, voltage, measuredPosition, measuredVelocity};
            for (double d : dataPacket) {
                data.add(d);
            }
        }
        // update desired motor voltage
        primaryMotorVoltage = motorVoltage;
    }

    public void reset() {
        super.reset();
        primaryMotorVoltage = 0.0;
    }

    boolean isWrongMechanism() {
        return !mechanism.equals("Arm")
                && !mechanism.equals("Elevator")
                && !mechanism.equals("Simple");
    }
}
