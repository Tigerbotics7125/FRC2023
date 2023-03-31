/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.lib;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class CANSparkMaxSim {

    private CANSparkMax mSparkMax;
    private SimDeviceSim mSparkMaxSim;
    private SimDouble mAppliedOutput;
    private SimInt mFaults;
    private SimInt mStickyFaults;
    private SimDouble mVelocity;
    private SimDouble mVelocityConversionFactor;
    private SimDouble mVelocityPreConversion;
    private SimDouble mPosition;
    private SimDouble mPosititonConversionFactor;
    private SimInt mMotorTemperature;
    private SimDouble mBusVoltage;
    private SimDouble mMotorCurrent;
    private SimDouble mAnalogVoltage;
    private SimDouble mAnalogVelocity;
    private SimDouble mAnalogPosition;
    private SimDouble mAltEncoderVelocity;
    private SimDouble mAltEncoderPosition;
    private SimInt mControlMode;
    private SimDouble mStallTorque;
    private SimDouble mFreeSpeed;
    private SimInt mFWVersion;

    public CANSparkMaxSim(CANSparkMax sparkMax) {
        mSparkMax = sparkMax;
        mSparkMaxSim = new SimDeviceSim("SPARK MAX", mSparkMax.getDeviceId());
        mAppliedOutput = mSparkMaxSim.getDouble("Applied Output");
        mFaults = mSparkMaxSim.getInt("Faults");
        mStickyFaults = mSparkMaxSim.getInt("Sticky Faults");
        mVelocity = mSparkMaxSim.getDouble("Velocity");
        mVelocityConversionFactor = mSparkMaxSim.getDouble("Velocity Conversion Factor");
        mVelocityPreConversion = mSparkMaxSim.getDouble("Velocity Pre-Conversion");
        mPosition = mSparkMaxSim.getDouble("Position");
        mPosititonConversionFactor = mSparkMaxSim.getDouble("Position Conversion Factor");
        mMotorTemperature = mSparkMaxSim.getInt("Motor Temperature");
        mBusVoltage = mSparkMaxSim.getDouble("Bus Voltage");
        mMotorCurrent = mSparkMaxSim.getDouble("Motor Current");
    }

    public double getAppliedOutput() {
        return mAppliedOutput.get();
    }

    public void setAppliedOutput(double value) {
        mAppliedOutput.set(value);
    }
}
