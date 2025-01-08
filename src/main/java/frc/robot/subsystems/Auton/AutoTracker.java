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
        // try {
        //     scoringPath = PathPlannerPath.fromPathFile("R10B");
        //     commands.addCommands(AutoBuilder.pathfindThenFollowPath(scoringPath, constraints));
        //     commands.addCommands(Commands.print("Scoring here"));
        //     commands.addCommands(Commands.waitSeconds(2));
        // } catch (Exception e) {}
        for (AutoSector autoSector : paths) {
            try {
                intakingpath = PathPlannerPath.fromPathFile(autoSector.intakingPath());
                scoringPath = PathPlannerPath.fromPathFile(autoSector.ShootingPath());
                addCommands(AutoBuilder.pathfindThenFollowPath(intakingpath, constraints));
                addCommands(Commands.print("Grab da tube"));
                // commands.addCommands(Commands.waitSeconds(2));
                addCommands(AutoBuilder.pathfindThenFollowPath(scoringPath, constraints));
                addCommands(Commands.print("Placing here"));
                // commands.addCommands(Commands.waitSeconds(2));

            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

}