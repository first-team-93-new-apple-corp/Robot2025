package frc.robot.subsystems.Auton;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.AutoSector;
import frc.robot.Constants.AutoConstants.IntakingStrategy;

public class AutoTracker extends SequentialCommandGroup {

    PathPlannerPath intakingpath;
    PathPlannerPath scoringPath;

    PathConstraints constraints = AutoConstants.constraints;

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

    private Command L2() {
        if (Utils.isSimulation()) {
            return Commands.print("to L4");
        } else {
            return subsystems.armSubsystem().Commands.L2().alongWith(subsystems.elevatorSubsystem().Commands.L2());
        }
    }

    private Command L4() {
        if (Utils.isSimulation()) {
            return Commands.print("to L4");
        } else {
            return subsystems.armSubsystem().Commands.L3().alongWith(subsystems.elevatorSubsystem().Commands.L3());
        }
    }

    private Command L1() {
        if (Utils.isSimulation()) {
            return Commands.print("to L1");
        } else {

            return subsystems.armSubsystem().Commands.L1().alongWith(subsystems.elevatorSubsystem().Commands.L1());
        }
    }

    private Command Intake(IntakingStrategy strategy) {
        if (Utils.isSimulation()) {
            return Commands.print("Intake");

        } else {
            // ground
            return ((subsystems.grabberSubsystem().Commands.intake()
                    .alongWith(subsystems.armSubsystem().Commands.GroundIntake())
                    .alongWith(subsystems.elevatorSubsystem().Commands.Bottom()))
                    .alongWith((subsystems.driveSubsystem().Commands
                            .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(.5))
                            .withDeadline(Commands.waitUntil(() -> SmartDashboard.getBoolean("Has Coral", true)))).andThen(subsystems.driveSubsystem().Commands
                                    .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0))))
                    .withDeadline(Commands.waitSeconds(1.6)))
                    .andThen((subsystems.driveSubsystem().Commands
                            .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(-.5))
                            .withDeadline(Commands.waitSeconds(.5))).andThen(subsystems.driveSubsystem().Commands
                                    .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)))
                            .withDeadline(Commands.waitSeconds(.7)));
        }
    }

    private Command Outtake() {
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
        boolean coral = SmartDashboard.getBoolean("Has Coral", false);
        addCommands(Commands.print("Auto Time"));
        addCommands(Commands.runOnce(() -> subsystems.driveSubsystem().resetPose(initalPose.get()),
                subsystems.driveSubsystem()));
                
        try {
            // preload
            if (!(preLoad.equals("x"))) {
                scoringPath = PathPlannerPath.fromPathFile(preLoad);
                addCommands(score(scoringPath));

                // coral = SmartDashboard.getBoolean("Has Coral", coral);
                // addCommands(AutoBuilder.pathfindThenFollowPath(scoringPath, constraints)
                // .alongWith(L4()));
                // addCommands(Outtake());
                // addCommands((subsystems.driveSubsystem().Commands
                // .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(-.5))
                // .withDeadline(Commands.waitSeconds(.75))).andThen(subsystems.driveSubsystem().Commands
                // .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)))
                // .withDeadline(Commands.waitSeconds(1)));
                // coral = SmartDashboard.getBoolean("Has Coral", coral);
                // if (!coral) {
                // addCommands(Commands.print("Scored Succesfully"));
                // } else {
                // addCommands(AutoBuilder.pathfindThenFollowPath(scoringPath, constraints)
                // .alongWith(L4()));
                // addCommands(Outtake());
                // addCommands((subsystems.driveSubsystem().Commands
                // .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(-.5))
                // .withDeadline(Commands.waitSeconds(.75))).andThen(subsystems.driveSubsystem().Commands
                // .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)))
                // .withDeadline(Commands.waitSeconds(1)));
                // }

            }

        } catch (Exception e) {
            e.printStackTrace();
        }

        for (AutoSector autoSector : paths) {
            try {
                intakingpath = PathPlannerPath.fromPathFile(autoSector.intakingPath());
                scoringPath = PathPlannerPath.fromPathFile(autoSector.ShootingPath());

                addCommands(Commands.print("going to intake [" + coral + "]"));
                addCommands(AutoBuilder.pathfindThenFollowPath(intakingpath, constraints));
                addCommands(Commands.waitSeconds(0.5).alongWith(L1()));
                addCommands(Intake(IntakingStrategy.ground));
                addCommands(Commands.print("intake"));

                addCommands((subsystems.driveSubsystem().Commands
                        .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(-.5))
                        .withDeadline(Commands.waitSeconds(.75))).andThen(subsystems.driveSubsystem().Commands
                                .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)))
                        .withDeadline(Commands.waitSeconds(1)));

                addCommands(score(scoringPath));

            } catch (Exception e) {
                e.printStackTrace();

            }
        }
    }

    // really coolio Code goes here (Ive got a plan)
    public void addSector(AutoSector sector) {
        try {
            intakingpath = PathPlannerPath.fromPathFile(sector.intakingPath());
            scoringPath = PathPlannerPath.fromPathFile(sector.ShootingPath());

            addCommands(AutoBuilder.pathfindThenFollowPath(intakingpath, constraints));
            addCommands(Commands.waitSeconds(0.5).alongWith(L1()));

            addCommands(Intake(IntakingStrategy.ground));
            addCommands(Commands.print("intake"));

            addCommands(AutoBuilder.pathfindThenFollowPath(scoringPath, constraints)
                    .alongWith(L4()));
            addCommands(Outtake());
            addCommands(((subsystems.driveSubsystem().Commands
                    .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(-.5))
                    .withDeadline(Commands.waitSeconds(.75))).andThen(subsystems.driveSubsystem().Commands
                            .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)))
                    .withDeadline(Commands.waitSeconds(1))));
        } catch (Exception e) {
            e.printStackTrace();

        }
    }

    public Command score(PathPlannerPath scoringPath) {
        return AutoBuilder.pathfindThenFollowPath(scoringPath, constraints)
                .alongWith(L4())
                .andThen(Outtake())
                .andThen((subsystems.driveSubsystem().Commands
                        .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(-.6))
                        .withDeadline(Commands.waitSeconds(.75))).andThen(subsystems.driveSubsystem().Commands
                                .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)))
                        .withDeadline(Commands.waitSeconds(1)))
                .andThen(new ConditionalCommand(Commands.print("Scored Succesfully"),

                        ((AutoBuilder.pathfindThenFollowPath(scoringPath, constraints).alongWith(L4())))
                                .andThen(Outtake())
                                .andThen((subsystems.driveSubsystem().Commands
                                        .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(-.5))
                                        .withDeadline(Commands.waitUntil(() -> SmartDashboard.getBoolean("Has Coral", true))))
                                        .andThen(subsystems.driveSubsystem().Commands
                                                .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)))
                                        .withDeadline(Commands.waitSeconds(1))),
                        () -> !SmartDashboard.getBoolean("Has Coral", true)));

    }
}