package frc.robot.subsystems.Auton;

import java.util.ArrayList;
import java.util.List;

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
    Autos.add(straight());
    Autos.add(Coral3());
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
    paths.add(new AutoSector("TestingPath", "testing part2"));

    AutoTracker tracker = new AutoTracker(subsystems, paths, () -> new Pose2d());
    return new Auto("straight", tracker.asCommand(), new Pose2d());
  }
  public Auto Coral3(){
    List<AutoSector> paths = new ArrayList<>();
    paths.add(new AutoSector("SC1", "R8A"));
    paths.add(new AutoSector("SC2", "R6A"));

    AutoTracker tracker = new AutoTracker(subsystems, paths, () -> PositionConstants.startingPoses.top());
    return new Auto("Three Coral", tracker.asCommand(), PositionConstants.startingPoses.top());

  }

}