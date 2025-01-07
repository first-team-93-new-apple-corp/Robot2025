package frc.robot.subsystems.Auton;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoDirector {
  AutoSubsystems subsystems;
  SendableChooser<Auto> autoChooser = new SendableChooser<Auto>();
  List<Auto> Autos = new ArrayList<>();

  public AutoDirector(AutoSubsystems subsystems) {
    this.subsystems = subsystems;
    AddAutos();
  }

  public record Auto(String name, Command command, Pose2d initPose) {}


  public Auto selection() {
      return autoChooser.getSelected();
  }

  public void AddAutos() {
    autoChooser.setDefaultOption(doNothing().name, doNothing());
    Autos.add(Coral());
    Autos.add(straight());
    Autos.add(PathPlannerAuto());
    for (Auto auto : Autos) {
      autoChooser.addOption(auto.name, auto);
    }
    SmartDashboard.putData("AutoChooser", autoChooser);
  }

  // ------------------------------------------Autos------------------------------------------
  public Auto doNothing() {
    return new Auto("doNothing", new InstantCommand(), new Pose2d());
  }

  public Auto straight(){
    List<AutoSector> paths = new ArrayList<>();
    paths.add(new AutoSector("TestingPath", "Test 2"));

    AutoTracker tracker = new AutoTracker(subsystems, paths, () -> new Pose2d());
    return new Auto("straight", tracker, new Pose2d());
  }

  public Auto Coral() {
    List<AutoSector> paths = new ArrayList<>();
    paths.add(new AutoSector("R6A", "SC1"));
    AutoTracker tracker = new AutoTracker(subsystems, paths, () -> PositionConstants.startingPoses.top());

    return new Auto("Speaker3", tracker, PositionConstants.startingPoses.top() );
  }

  public Auto PathPlannerAuto(){
    return new Auto("PP Auto", AutoBuilder.buildAuto("New Auto"), new Pose2d());
  }

}