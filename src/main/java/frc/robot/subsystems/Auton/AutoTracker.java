package frc.robot.subsystems.Auton;

import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve.TunerConstants;

public class AutoTracker extends SequentialCommandGroup {

    PathPlannerPath intakingpath;
    PathPlannerPath scoringPath;

    private LinearVelocity MaxSpeed = MetersPerSecond.of(4.73);// kSpeedAt12Volts desired top speed
    private AngularVelocity MaxAngularRate = RadiansPerSecond.of(11.887); // 3/4 of a rotation per second
    private LinearAcceleration MaxAcceleration = MetersPerSecondPerSecond.of(9.8);
    private AngularAcceleration MaxAngularAcceleration = DegreesPerSecondPerSecond.of(1290);
    
    PathConstraints constraints = new PathConstraints(MaxSpeed.div(9), MaxAcceleration.div(12), MaxAngularRate.div(9), MaxAngularAcceleration.div(9));


    private AutoSubsystems subsystems;

    public AutoTracker(AutoSubsystems subsystems, List<AutoSector> paths, Supplier<Pose2d> initalPose, boolean Leave) {
        this(subsystems, paths, initalPose, "x", Leave);
    }

    public AutoTracker(AutoSubsystems subsystems, List<AutoSector> paths, Supplier<Pose2d> initalPose) {
        this(subsystems, paths, initalPose, "x", true);
    }

    public AutoTracker(AutoSubsystems subsystems, List<AutoSector> paths, Supplier<Pose2d> initalPose, String preLoad) {
        this(subsystems, paths, initalPose, preLoad, true);
    }

    Command L2() {
        if (Utils.isSimulation()) {
            return Commands.print("to L4");
        } else {
            return subsystems.armSubsystem().Commands.L2().alongWith(subsystems.elevatorSubsystem().Commands.L2());
        }
    }
    Command L4() {
        if (Utils.isSimulation()) {
            return Commands.print("to L4");
        } else {
            return subsystems.armSubsystem().Commands.L2().alongWith(subsystems.elevatorSubsystem().Commands.L2());
        }
    }

    Command L1() {
        if (Utils.isSimulation()) {
            return Commands.print("to L1");
        } else {
            
            return subsystems.armSubsystem().Commands.L1().alongWith(subsystems.elevatorSubsystem().Commands.L1());
        }
    }

    Command Intake() {
        if (Utils.isSimulation()) {
            return Commands.print("Intake");

        } else {
            return Commands.print("Intake");
            // CommandScheduler.getInstance().clearComposedCommands();
            // return subsystems.intakeCommand();
        }
    }

    Command Outtake() {
        if (Utils.isSimulation()) {
            return Commands.print("outtake");

        } else {
            return (subsystems.elevatorSubsystem().Commands.outtake().alongWith(Commands.waitSeconds(1)))
                    .alongWith(subsystems.grabberSubsystem().Commands.outtake().withDeadline(Commands.waitSeconds(1)));
        }
    }

    public AutoTracker(AutoSubsystems subsystems, List<AutoSector> paths, Supplier<Pose2d> initalPose, String preLoad,
            boolean Leave) {
        this.subsystems = subsystems;

        addCommands(Commands.print("Auto Time"));
        addCommands(Commands.runOnce(() -> subsystems.driveSubsystem().resetPose(initalPose.get()),
                subsystems.driveSubsystem()));

        try {
            // preload
            if (!(preLoad.equals("x"))) {
                scoringPath = PathPlannerPath.fromPathFile(preLoad);
                addCommands(AutoBuilder.pathfindThenFollowPath(scoringPath, constraints)
                        .alongWith(L4()));
                addCommands(Outtake());
            }

        } catch (Exception e) {
            e.printStackTrace();
        }

        for (AutoSector autoSector : paths) {
            try {
                intakingpath = PathPlannerPath.fromPathFile(autoSector.intakingPath());
                scoringPath = PathPlannerPath.fromPathFile(autoSector.ShootingPath());

                addCommands(AutoBuilder.pathfindThenFollowPath(intakingpath, constraints)
                        .alongWith(Commands.waitSeconds(.5).andThen(L1())));
                addCommands(Intake());
                addCommands(Commands.print("intake"));
                addCommands(AutoBuilder.pathfindThenFollowPath(scoringPath, constraints)
                        .alongWith(L4()));
                addCommands(Outtake());

            } catch (Exception e) {
                e.printStackTrace();

            }
        }
    }

}