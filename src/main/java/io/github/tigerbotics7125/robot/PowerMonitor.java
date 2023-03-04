/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import io.github.tigerbotics7125.lib.RollingAverage;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger;

public class PowerMonitor {

    private static final PowerMonitor kInstance = new PowerMonitor();

    public static final PowerMonitor getInstance() {
        return kInstance;
    }

    private PowerDistribution mPDH;

    private PowerMonitor() {
        mPDH = new PowerDistribution();
    }

    public double getCurrent(int channel) {
        return mPDH.getCurrent(channel);
    }

    public Trigger getSpike(int channel, double thresholdOverAverageAmps) {
        RollingAverage avg = new RollingAverage(3);
        return new Trigger(() -> avg.calculate(getCurrent(channel)) > thresholdOverAverageAmps);
    }
}
