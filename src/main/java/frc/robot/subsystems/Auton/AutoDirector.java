package frc.robot.subsystems.Auton;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AutoConstants.AutoSector;
import frc.robot.Constants.AutoConstants.AutoSectorV2;
import frc.robot.subsystems.Auton.PositionConstants.GamePeice;
import frc.robot.subsystems.Auton.PositionConstants.HumanPlayerIntake;
import frc.robot.subsystems.Auton.PositionConstants.Reef;
import frc.robot.subsystems.Auton.PositionConstants.startingPoses;

public class AutoDirector {
  AutoSubsystems subsystems;
  SendableChooser<Auto> autoChooser = new SendableChooser<Auto>();
  List<Auto> Autos = new ArrayList<>();

  public AutoDirector(AutoSubsystems subsystems) {
    this.subsystems = subsystems;
    AddAutos();
  }

  public record Auto(String name, Command command, Pose2d initPose) {
    public Auto(String name, Command command) {
      this(name, command, new Pose2d());
    }
  }

  public Auto selection() {

    if (autoChooser.getSelected().name == dummyAuto().name) {
      return SmartAuto();
    }
    return autoChooser.getSelected();
  }

  public void AddAutos() {
    autoChooser.setDefaultOption(doNothing().name, doNothing());
    // Autos.add(Coral());
    // Autos.add(straight());
    // Autos.add(PathPlannerAuto());
    // Autos.add(HPintake());
    // Autos.add(newAuto());
    // Autos.add(Leftside());
    // Autos.add(RightSide());
    // Autos.add(CenterSide());
    // Autos.add(CenterSide3());
    // Autos.add(CenterSideV2());
    // Autos.add(CenterSideV3());
    // Autos.add(testHP());
    // Autos.add(dummyAuto());
    Autos.add(RightSide());
    Autos.add(LeftSide());
    Autos.add(Middle());
    setupSmartAuto();
    for (Auto auto : Autos) {
      autoChooser.addOption(auto.name, auto);
    }
    SmartDashboard.putData("AutoChooser", autoChooser);
  }

  // ------------------------------------------Autos------------------------------------------
  // public Auto doNothing() {
  //   return new Auto("doNothing", new InstantCommand(), new Pose2d());
  // }
  public Auto doNothing() {
    Command moveXPos = (subsystems.driveSubsystem().Commands.applyRequest(() ->  new SwerveRequest.RobotCentric().withVelocityX(1)));
    Command speedZero = subsystems.driveSubsystem().Commands.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0));
    Command cmd = moveXPos.withDeadline(Commands.waitSeconds(1)).andThen(speedZero);

    // var cmd = subsystems.driveSubsystem().setControl(subsystems.driveSubsystem().Commands.applyRequest(() ->  new SwerveRequest.RobotCentric().withVelocityX(1)));
    return new Auto("doNothing", cmd, new Pose2d());
  }
  public Auto newAuto() {
    List<AutoSector> paths = new ArrayList<>();
    paths.add(new AutoSector("TSC", "R6B"));

    AutoTracker tracker = new AutoTracker(subsystems, paths, () -> new Pose2d());

    return new Auto("new Auto", tracker, new Pose2d());
  }

  public Auto straight() {
    List<AutoSector> paths = new ArrayList<>();
    paths.add(new AutoSector("TestingPath", "Test 2"));

    AutoTracker tracker = new AutoTracker(subsystems, paths, () -> new Pose2d(), false);
    return new Auto("straight", tracker, new Pose2d());
  }

  public Auto Coral() {
    List<AutoSector> paths = new ArrayList<>();
    paths.add(new AutoSector("C1", "R6B"));
    paths.add(new AutoSector("C2", "R4A"));
    paths.add(new AutoSector("C3", "R6A"));

    AutoTracker tracker = new AutoTracker(subsystems, paths, () -> PositionConstants.startingPoses.Left());

    return new Auto("Coral", tracker, PositionConstants.startingPoses.Left());
  }

  public Auto HPintake() {
    List<AutoSector> paths = new ArrayList<>();
    paths.add(new AutoSector("TSC", "R8B"));
    paths.add(new AutoSector("TSC", "R8A"));
    paths.add(new AutoSector("TSC", "R6B"));
    paths.add(new AutoSector("BSC", "R6A"));
    paths.add(new AutoSector("BSC", "R4A"));
    paths.add(new AutoSector("BSC", "R4B"));

    AutoTracker tracker = new AutoTracker(subsystems, paths, () -> PositionConstants.startingPoses.Left());

    return new Auto("Human Player Intake", tracker, PositionConstants.startingPoses.Left());
  }

  public Auto PathPlannerAuto() {
    return new Auto("PP Auto", AutoBuilder.buildAuto("New New Auto"), new Pose2d());
  }

  public Auto Leftside() {
    List<AutoSector> paths = new ArrayList<>();
    paths.add(new AutoSector("LHP", "R10B"));
    paths.add(new AutoSector("LHP", "R10A"));
    paths.add(new AutoSector("LHP", "R8B"));

    AutoTracker tracker = new AutoTracker(subsystems, paths, () -> PositionConstants.startingPoses.Left(), "R12A");

    return new Auto("Leftside 4 Coral", tracker, PositionConstants.startingPoses.Left());
  }

  

  public Auto CenterSide() {
    List<AutoSector> paths = new ArrayList<>();
    paths.add(new AutoSector("C1", "R6B"));
    paths.add(new AutoSector("C2", "R6A"));
    paths.add(new AutoSector("C3", "R4B"));

    AutoTracker tracker = new AutoTracker(subsystems, paths, () -> PositionConstants.startingPoses.Left(), "R8A");

    return new Auto("CenterSide 4 Coral", tracker, PositionConstants.startingPoses.Left());
  }

  public Auto CenterSide3() {
    List<AutoSector> paths = new ArrayList<>();
    paths.add(new AutoSector("C2", "R6B"));
    paths.add(new AutoSector("C1", "R8A"));

    AutoTracker tracker = new AutoTracker(subsystems, paths, () -> PositionConstants.startingPoses.CenterLeft(), "R6A");

    return new Auto("CenterSide 3 Coral", tracker, PositionConstants.startingPoses.CenterLeft());
  }

  public Auto CenterSideV2() {
    AutoTrackerV2 tracker = new AutoTrackerV2(subsystems, () -> PositionConstants.startingPoses.CenterLeft());
    tracker.addPreload("R6A");
    tracker.addSector(new AutoSector("C1", "R6B"));
    tracker.addSector(new AutoSector("C2", "R8A"));

    return new Auto("CenterSide 3 Coral V2 - [new tracker]", tracker);
  }

  public Auto CenterSideV3() {
    AutoTrackerV2 tracker = new AutoTrackerV2(subsystems, () -> PositionConstants.startingPoses.Center());
    tracker.addPoint(new Pose2d(1.848, 4.934, PositionConstants.awayFromAlliance));
    tracker.addPreload(Reef.BlueR6B);
    tracker.addSector(new AutoSectorV2(GamePeice.BlueC2, Reef.BlueR6A));
    tracker.addSector(new AutoSectorV2(GamePeice.BlueC1, Reef.BlueR8A));

    return new Auto("CenterSide 3 Coral V3 - [Pose Pathfinding]", tracker);
  }

  public Auto testHP() {
    AutoTrackerV2 tracker = new AutoTrackerV2(subsystems, () -> PositionConstants.startingPoses.CenterLeft());
    tracker.addSector(new AutoSectorV2(HumanPlayerIntake.BlueLeft, Reef.BlueR8A));
    return new Auto("Testing HP intake", tracker);
  }

  public Auto RightSide(){
    AutoTrackerV2 tracker = new AutoTrackerV2(subsystems, ()-> PositionConstants.startingPoses.RightMidBarge());
    tracker.addPoint(new Pose2d(1.232, 3.071, PositionConstants.awayFromAlliance));
    tracker.addPreload(Reef.BlueR4A);
    tracker.addSector(new AutoSectorV2(GamePeice.BlueC3, Reef.BlueR4B));
    tracker.addSector(new AutoSectorV2(GamePeice.BlueC2, Reef.BlueR6A));

    return new Auto("RightSIde", tracker);
  }

  public Auto LeftSide(){
    AutoTrackerV2 tracker = new AutoTrackerV2(subsystems, ()-> PositionConstants.startingPoses.LeftBargeMiddle());
    tracker.addPoint(new Pose2d(1.848, 4.934, PositionConstants.awayFromAlliance));
    tracker.addPreload(Reef.BlueR8A);
    tracker.addSector(new AutoSectorV2(GamePeice.BlueC1, Reef.BlueR6B));
    tracker.addSector(new AutoSectorV2(GamePeice.BlueC2, Reef.BlueR8A));

    return new Auto("LeftSide", tracker);
  }

  public Auto Middle(){
    AutoTrackerV2 tracker = new AutoTrackerV2(subsystems, ()-> PositionConstants.startingPoses.Center());
    tracker.addPreload(Reef.BlueR12B);

    return new Auto("Middle", tracker);
  }

  public Auto auto(String name, List<AutoSector> paths, Supplier<Pose2d> initalPose,
      String preLoadPath, boolean Leave) {

    AutoTracker tracker = new AutoTracker(subsystems, paths, initalPose, preLoadPath, Leave);

    return new Auto(name, tracker, initalPose.get());
  }

  public Auto auto(String name, List<AutoSector> paths, Supplier<Pose2d> initalPose,
      boolean Leave) {

    AutoTracker tracker = new AutoTracker(subsystems, paths, initalPose, Leave);

    return new Auto(name, tracker, initalPose.get());
  }

  public Auto dummyAuto() {
    return new Auto("SmartDashboard Auto", new InstantCommand());
  }

  private void setupSmartAuto() {
    SendableChooser<Pose2d> startingPose = new SendableChooser<Pose2d>();
    startingPose.setDefaultOption("Left", startingPoses.Left());
    startingPose.addOption("Right", startingPoses.right());
    startingPose.addOption("Center", startingPoses.CeneterCoral());
    startingPose.addOption("Center Left", startingPoses.CenterLeft());
    SmartDashboard.putData("SmartAuto/Starting Pose", startingPose);

    SmartDashboard.putNumber("SmartAuto/Amount to Score", 1);
    SendableChooser<String> preload = new SendableChooser<String>();
    preload.setDefaultOption("R2A", "R2A");
    preload.addOption("R2B", "R2B");
    preload.addOption("R4A", "R4A");
    preload.addOption("R4B", "R4B");
    preload.addOption("R6A", "R6A");
    preload.addOption("R6B", "R6B");
    preload.addOption("R8A", "R8A");
    preload.addOption("R8B", "R8B");
    preload.addOption("R10A", "R10A");
    preload.addOption("R10B", "R10B");
    preload.addOption("R12A", "R12A");
    preload.addOption("R12B", "R12B");
    SmartDashboard.putData("SmartAuto/preload", preload);

    SendableChooser<String> score1 = new SendableChooser<>();
    score1.setDefaultOption("R2A", "R2A");
    score1.addOption("R2B", "R2B");
    score1.addOption("R4A", "R4A");
    score1.addOption("R4B", "R4B");
    score1.addOption("R6A", "R6A");
    score1.addOption("R6B", "R6B");
    score1.addOption("R8A", "R8A");
    score1.addOption("R8B", "R8B");
    score1.addOption("R10A", "R10A");
    score1.addOption("R10B", "R10B");
    score1.addOption("R12A", "R12A");
    score1.addOption("R12B", "R12B");

    SendableChooser<String> score2 = new SendableChooser<>();
    score2.setDefaultOption("R2A", "R2A");
    score2.addOption("R2B", "R2B");
    score2.addOption("R4A", "R4A");
    score2.addOption("R4B", "R4B");
    score2.addOption("R6A", "R6A");
    score2.addOption("R6B", "R6B");
    score2.addOption("R8A", "R8A");
    score2.addOption("R8B", "R8B");
    score2.addOption("R10A", "R10A");
    score2.addOption("R10B", "R10B");
    score2.addOption("R12A", "R12A");
    score2.addOption("R12B", "R12B");

    SendableChooser<String> score3 = new SendableChooser<>();
    score3.setDefaultOption("R2A", "R2A");
    score3.addOption("R2B", "R2B");
    score3.addOption("R4A", "R4A");
    score3.addOption("R4B", "R4B");
    score3.addOption("R6A", "R6A");
    score3.addOption("R6B", "R6B");
    score3.addOption("R8A", "R8A");
    score3.addOption("R8B", "R8B");
    score3.addOption("R10A", "R10A");
    score3.addOption("R10B", "R10B");
    score3.addOption("R12A", "R12A");
    score3.addOption("R12B", "R12B");

    SmartDashboard.putData("SmartAuto/Score1", score1);
    SmartDashboard.putData("SmartAuto/Score2", score2);
    SmartDashboard.putData("SmartAuto/Score3", score3);

    SendableChooser<String> Intake1 = new SendableChooser<String>();
    Intake1.setDefaultOption("LHP", "LHP");
    Intake1.addOption("RHP", "RHP");
    Intake1.addOption("C1", "C1");
    Intake1.addOption("C2", "C2");
    Intake1.addOption("C3", "C3");
    SmartDashboard.putData("SmartAuto/Intake1", Intake1);

    SendableChooser<String> Intake2 = new SendableChooser<String>();
    Intake2.setDefaultOption("LHP", "LHP");
    Intake2.addOption("RHP", "RHP");
    Intake2.addOption("C1", "C1");
    Intake2.addOption("C2", "C2");
    Intake2.addOption("C3", "C3");

    SendableChooser<String> Intake3 = new SendableChooser<String>();
    Intake3.setDefaultOption("LHP", "LHP");
    Intake3.addOption("RHP", "RHP");
    Intake3.addOption("C1", "C1");
    Intake3.addOption("C2", "C2");
    Intake3.addOption("C3", "C3");

    SmartDashboard.putData("SmartAuto/Intake2", Intake2);
    SmartDashboard.putData("SmartAuto/Intake3", Intake3);

  }

  public Auto SmartAuto() {
    AutoTrackerV2 tracker = new AutoTrackerV2(subsystems, () -> PositionConstants.startingPoses.CenterLeft());

    tracker.addPreloadV2(SmartDashboard.getString("SmartAuto/preload", "x"));

    for (int i = 0; i < SmartDashboard.getNumber("SmartAuto/Amount to Score", 1); i++) {
      tracker.addSectorV2(new AutoSector(SmartDashboard.getString("SmartAuto/Intake" + (i + 1), "LHP"),
          SmartDashboard.getString("SmartAuto/Score" + (i + 1), "R2A")));
    }
    return new Auto("SmartDashboard Auto", tracker);
  }
}