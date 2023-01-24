/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.sysid.logging;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.tigerbotics7125.robot.Robot;
import java.util.ArrayList;
import java.util.List;

/** Add your docs here. */
public class SysIdLogger {
    String mechanism = "";
    String test = "";
    String testType = "";
    boolean rotate = false;
    double voltageCommand = 0.0;
    double motorVoltage = 0.0;
    double startTime = 0.0;
    double timestamp = 0.0;
    double ackNum = 0;
    final int dataVectorSize = 36000;
    ArrayList<Double> data;

    public SysIdLogger() {
        System.out.println("Initializing logger");
        data = new ArrayList<>(dataVectorSize);
        LiveWindow.disableAllTelemetry();
    }

    public void initLogging() {
        mechanism = SmartDashboard.getString("SysIdTest", "");
        if (isWrongMechanism()) {
            SmartDashboard.putBoolean("SysIdWrongMech", true);
        }
        testType = SmartDashboard.getString("SysIdTestType", "");
        rotate = SmartDashboard.getBoolean("SysIdRotate", false);
        voltageCommand = SmartDashboard.getNumber("SysIdVoltageCommand", 0.0);
        startTime = Timer.getFPGATimestamp();
        data.clear();
        SmartDashboard.putString("SysIdTelemetry", "");
        ackNum = SmartDashboard.getNumber("SysIdAckNumber", 0);
    }

    public double measureVoltage(List<CANSparkMax> controllers) {
        double sum = 0.0;
        for (int i = 0; i < controllers.size(); ++i) {
            CANSparkMax motor = controllers.get(i);
            sum += motor.getBusVoltage() * motor.getAppliedOutput();
            if (Robot.isSimulation()) {
                System.out.println("Recording SparkMax Voltage\n");
            }
        }
        return sum / controllers.size();
    }

    public void sendData() {
        System.out.println("Collected: " + data.size() + " data points.\n");

        SmartDashboard.putBoolean("SysIdOverflow", data.size() >= dataVectorSize);
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < data.size(); ++i) {
            sb.append(data.get(i));
            if (i < data.size() - 1) {
                sb.append(",");
            }
        }

        String type = testType.equals("Dynamic") ? "fast" : "slow";
        String direction = voltageCommand > 0 ? "forward" : "backward";
        String test = new StringBuilder().append(type).append("-").append(direction).toString();

        SmartDashboard.putString(
                "SysIdTelemetry",
                new StringBuilder().append(test).append(";").append(sb.toString()).toString());
        SmartDashboard.putNumber("SysIdAckNumber", ++ackNum);

        reset();
    }

    public void clearWhenRecieved() {
        if (SmartDashboard.getNumber("SysIdAckNumber", 0.0) > ackNum) {
            SmartDashboard.putString("SysIdTelemetry", "");
            ackNum = SmartDashboard.getNumber("SysIdAckNumber", 0.0);
        }
    }

    public void updateThreadPriority() {
        if (!Robot.isSimulation()) {
            if (!Notifier.setHALThreadPriority(true, 40)
                    || Threads.setCurrentThreadPriority(true, 15)) {
                throw new RuntimeException("Setting RT priority failed");
            }
        }
    }

    public void updateData() {
        timestamp = Timer.getFPGATimestamp();
        // if the mechanism is supported
        if (!isWrongMechanism()) {
            // if ramp test
            if (testType.equals("Quasistatic")) {
                // voltage = ramp rate * test elapsed time
                motorVoltage = voltageCommand * (timestamp - startTime);
            } else if (testType.equals("Dynamic")) { // if steady state test
                // voltage = steady state voltage
                motorVoltage = voltageCommand;
            } else {
                motorVoltage = 0.0;
            }
        } else {
            motorVoltage = 0.0;
        }
    }

    public void reset() {
        motorVoltage = 0.0;
        timestamp = 0.0;
        startTime = 0.0;
        data.clear();
    }

    public void setMotorControllers(double volts, List<CANSparkMax> motors) {
        for (CANSparkMax motor : motors) {
            motor.getPIDController().setReference(volts, ControlType.kVoltage);
        }
    }

    boolean isWrongMechanism() {
        return !mechanism.equals("Arm")
                && !mechanism.equals("Elevator")
                && !mechanism.equals("Simple");
    }
}
