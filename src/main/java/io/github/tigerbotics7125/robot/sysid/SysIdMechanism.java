/*
 * Copyright (c) 2022 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot.sysid;

import com.revrobotics.CANSparkMax;

public interface SysIdMechanism {

    public CANSparkMax getMotor();

    public double getPosition();

    public double getVelocity();
}
