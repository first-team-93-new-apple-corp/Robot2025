package frc.robot.subsystems.Auton;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AutoConstants.AutoSector;

public class AutoDirector {
  AutoSubsystems subsystems;
  SendableChooser<Auto> autoChooser = new SendableChooser<Auto>();
  List<Auto> Autos = new ArrayList<>();

  public AutoDirector(AutoSubsystems subsystems) {
    this.subsystems = subsystems;
    AddAutos();
  }

  public record Auto(String name, Command command, Pose2d initPose) {
  }

  public Auto selection() {
    return autoChooser.getSelected();
  }

  public void AddAutos() {
    autoChooser.setDefaultOption(doNothing().name, doNothing());
    // Autos.add(Coral());
    // Autos.add(straight());
    // Autos.add(PathPlannerAuto());
    // Autos.add(HPintake());
    // Autos.add(newAuto());
    Autos.add(Leftside());
    Autos.add(RightSide());
    Autos.add(CenterSide());
    Autos.add(CenterSide2());
    Autos.add(Center3());   

    for (Auto auto : Autos) {
      autoChooser.addOption(auto.name, auto);
    }
    SmartDashboard.putData("AutoChooser", autoChooser);
  }

  // ------------------------------------------Autos------------------------------------------
  public Auto doNothing() {
    return new Auto("doNothing", new InstantCommand(), new Pose2d());
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

  public Auto RightSide() {
    List<AutoSector> paths = new ArrayList<>();
    paths.add(new AutoSector("RHP", "R2A"));
    paths.add(new AutoSector("RHP", "R2B"));
    paths.add(new AutoSector("RHP", "R4A"));

    AutoTracker tracker = new AutoTracker(subsystems, paths, () -> PositionConstants.startingPoses.right(), "R12B");

    return new Auto("Rightside 4 Coral", tracker, PositionConstants.startingPoses.right());
  }

  public Auto CenterSide() {
    List<AutoSector> paths = new ArrayList<>();
    paths.add(new AutoSector("C1", "R6B"));
    paths.add(new AutoSector("C2", "R6A"));
    paths.add(new AutoSector("C3", "R4B"));

    AutoTracker tracker = new AutoTracker(subsystems, paths, () -> PositionConstants.startingPoses.Left(), "R8A");

    return new Auto("CenterSide 4 Coral", tracker, PositionConstants.startingPoses.Left());
  }

  public Auto CenterSide2() {
    List<AutoSector> paths = new ArrayList<>();
    paths.add(new AutoSector("C2", "R6B"));
    paths.add(new AutoSector("C1", "R8A"));

    AutoTracker tracker = new AutoTracker(subsystems, paths, () -> PositionConstants.startingPoses.CenterLeft(), "R6A");
    
    return new Auto("CenterSide 2 Coral", tracker, PositionConstants.startingPoses.CenterLeft());
  }
  public Auto Center3() {
    List<AutoSector> paths = new ArrayList<>();
    paths.add(new AutoSector("C2", "R6B"));
    paths.add(new AutoSector("C1", "R8A"));

    AutoTracker tracker = new AutoTracker(subsystems, paths, () -> PositionConstants.startingPoses.Left(), "R6A");
    
    return new Auto("CenterSide 3 Coral", tracker, PositionConstants.startingPoses.Left());
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
}