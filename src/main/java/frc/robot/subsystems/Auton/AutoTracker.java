package frc.robot.subsystems.Auton;

import java.util.List;
import java.util.function.Supplier;
import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve.TunerConstants;

public class AutoTracker extends SequentialCommandGroup {
    PathPlannerPath intakingpath;
    PathPlannerPath scoringPath;
    PathConstraints constraints = new PathConstraints(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond), 9.8   , 4, 27.92527);
    // Vision m_Vision = new Vision();

    public AutoTracker( AutoSubsystems subsystems, List<AutoSector> paths, Supplier<Pose2d> initalPose){
        addCommands(Commands.print("Auto Time"));
        addCommands(Commands.runOnce(() -> subsystems.driveSubsystem().resetPose(initalPose.get()), subsystems.driveSubsystem()));
        try {
            //preload
        
            scoringPath = PathPlannerPath.fromPathFile("R10B");
            addCommands(AutoBuilder.pathfindThenFollowPath(scoringPath, constraints).alongWith(Commands.print("Raising the Elevator")));
            addCommands(Commands.print("Placing here"));
            addCommands(Commands.waitSeconds(1));
        } catch (Exception e) {}
        for (AutoSector autoSector : paths) {
            try {
                intakingpath = PathPlannerPath.fromPathFile(autoSector.intakingPath());
                scoringPath = PathPlannerPath.fromPathFile(autoSector.ShootingPath());
                addCommands(AutoBuilder.pathfindThenFollowPath(intakingpath, constraints).alongWith(Commands.print("Lowering the Elevator")));
                addCommands(Commands.print("Grab da tube"));
                addCommands(Commands.waitSeconds(1));
                addCommands(AutoBuilder.pathfindThenFollowPath(scoringPath, constraints).alongWith(Commands.print("Raising the Elevator")));
                addCommands(Commands.print("Placing here"));
                addCommands(Commands.waitSeconds(1));

            } catch (Exception e) {}
        }
    }

}