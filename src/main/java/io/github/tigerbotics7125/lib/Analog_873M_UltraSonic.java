package io.github.tigerbotics7125.lib;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Because the 873M returns 0-10V you need to use a https://en.wikipedia.org/wiki/Voltage_divider to get 0-5V that the rio can handle.
 *
 */
public class Analog_873M_UltraSonic {

    public enum SensorType {
        ANALOG_CURRENT,
        ANALOG_VOLTAGE,
    }

    private SensorType mSensorType;
    private AnalogInput mInput;

    public Analog_873M_UltraSonic(SensorType type, AnalogInput input) {
        mSensorType = type;
        mInput = input;
    }

    public double getPercentage() {
        return switch (mSensorType) {
            case ANALOG_VOLTAGE -> {
                double voltsInput = mInput.getVoltage();
                double voltsSensor = voltsInput * 2D;

                yield  10D / voltsSensor;
            }
            default -> 0.0;
        };
    }

}
