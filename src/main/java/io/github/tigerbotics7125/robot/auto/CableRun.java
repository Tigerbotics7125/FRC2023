package io.github.tigerbotics7125.robot.auto;

import java.util.List;
import java.util.Map;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;

public class CableRun implements Auto {

    @Override
    public List<PathPlannerTrajectory> getPath() {
        return PathPlanner.loadPathGroup("CableRun", new PathConstraints(2, .5));

    }

    @Override
    public Map<String, Command> getEventMap() {
        return Map.of();
        }

}
