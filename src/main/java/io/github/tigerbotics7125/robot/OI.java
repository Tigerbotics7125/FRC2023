/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package io.github.tigerbotics7125.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import io.github.tigerbotics7125.robot.constants.OIConstants;
import io.github.tigerbotics7125.tigerlib.input.controller.XboxController;
import io.github.tigerbotics7125.tigerlib.input.trigger.Trigger;

public class OI {

    public enum Controller {
        DRIVER,
        OPERATOR,
        BOTH;
    }

    public final XboxController mDriver;
    public final XboxController mOp;
    public final Trigger mRioUserButton;

    public OI() {
        mDriver = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
        mOp = new XboxController(OIConstants.OPERATOR_CONTROLLER_PORT);
        mRioUserButton = new Trigger(RobotController::getUserButton);
    }

    /**
     * @param controller Controller to rumble.
     * @param strength Strength [0, 1] to rumble at.
     * @param time Time in seconds to rumble for.
     * @return A Command which will rumble the designated controller at a given strength for a given
     *     amount of time.
     */
    public CommandBase rumble(Controller controller, double strength, double time) {
        if (controller == Controller.BOTH)
            return rumble(Controller.DRIVER, strength, time)
                    .alongWith(rumble(Controller.OPERATOR, strength, time));
        XboxController xbox =
                switch (controller) {
                    case DRIVER -> mDriver;
                    case OPERATOR -> mOp;
                    default -> null;
                };
        return Commands.runOnce(() -> xbox.setRumble(RumbleType.kBothRumble, strength))
                .andThen(Commands.waitSeconds(time))
                .andThen(Commands.runOnce(() -> xbox.setRumble(RumbleType.kBothRumble, 0)));
    }

    /**
     * @param controller Controller to rumble.
     * @return A Command which will rumble the designated controller for a defautl 75% strength for
     *     3/4 of a second.
     */
    public CommandBase rumble(Controller controller) {
        return rumble(controller, .75, .5);
    }
}
