package frc.robot.subsystems.Auton;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve.TunerConstants;

public class AutoTracker extends SequentialCommandGroup {
    PathPlannerPath intakingpath;
    PathPlannerPath scoringPath;
    private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    private AngularVelocity MaxAngularRate = RadiansPerSecond.of(11.887); // 3/4 of a rotation per second
    private LinearAcceleration MaxAcceleration = MetersPerSecondPerSecond.of(14.715);
    private AngularAcceleration MaxAngularAcceleration = RadiansPerSecondPerSecond.of(68.931);
    PathConstraints constraints = new PathConstraints(MaxSpeed, MaxAcceleration,
            MaxAngularRate, MaxAngularAcceleration); // Vision m_Vision = new Vision();

    public AutoTracker(AutoSubsystems subsystems, List<AutoSector> paths, Supplier<Pose2d> initalPose) {
        this(subsystems, paths, initalPose, "x",
                true);
    }

    public AutoTracker(AutoSubsystems subsystems, List<AutoSector> paths, Supplier<Pose2d> initalPose, String preLoad) {
        this(subsystems, paths, initalPose, preLoad,
                true);
    }

    public AutoTracker(AutoSubsystems subsystems, List<AutoSector> paths, Supplier<Pose2d> initalPose, String preLoad,
            boolean Leave) {

        Command ArmL4 = subsystems.armSubsystem().Commands.L4();
        Command ElevatorL4 = subsystems.elevatorSubsystem().Commands.L4();
        Command ElevatorOutake = subsystems.elevatorSubsystem().Commands.L3();
        Command GrabberOutake = subsystems.grabberSubsystem().Commands.outtake();

        addCommands(Commands.print("Auto Time"));
        addCommands(Commands.runOnce(() -> subsystems.driveSubsystem().resetPose(initalPose.get()),
                subsystems.driveSubsystem()));

        try {
            // preload
            if (!(preLoad.equals("x"))) {
                scoringPath = PathPlannerPath.fromPathFile("R10B");
                addCommands(AutoBuilder.pathfindThenFollowPath(scoringPath, constraints)
                        .alongWith(ElevatorL4).alongWith(ArmL4));
                addCommands(ElevatorOutake.alongWith(GrabberOutake));
            }

        } catch (Exception e) {}

        for (AutoSector autoSector : paths) {
            try {
                intakingpath = PathPlannerPath.fromPathFile(autoSector.intakingPath());
                scoringPath = PathPlannerPath.fromPathFile(autoSector.ShootingPath());

                addCommands(AutoBuilder.pathfindThenFollowPath(intakingpath, constraints)
                        .alongWith(Commands.print("Lowering the Elevator")));
                addCommands(Commands.print("Grab da tube"));
                addCommands(Commands.waitSeconds(.5));
                
                addCommands(AutoBuilder.pathfindThenFollowPath(scoringPath, constraints)
                        .alongWith(ElevatorL4).alongWith(ArmL4));
                addCommands(ElevatorOutake.alongWith(GrabberOutake));

            } catch (Exception e) {
            }
        }
    }

}