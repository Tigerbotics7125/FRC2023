/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.lib;

import edu.wpi.first.wpilibj.AnalogInput;

public class MB1122 {
    public static final double CENTIMETER_PER_VOLT = 0.00977;

    AnalogInput mSensor;

    public MB1122(AnalogInput sensorPort) {
        mSensor = sensorPort;
    }

    /**
     * @return The distance as returned by the sensor, actual distances less than .3m are not
     *     measured properly, and should not be considered, (ie: returned value of <.3m or ~.3m are
     *     invalid).
     */
    public double getDistanceMeters() {
        double volts = mSensor.getVoltage();
        double cm = volts / CENTIMETER_PER_VOLT;
        return cm / 100.0;
    }
}
