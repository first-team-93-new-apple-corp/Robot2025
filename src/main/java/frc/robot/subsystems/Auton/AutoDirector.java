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
  private AutoSector SG1 = new AutoSector("SG1", "SC");
  private AutoSector SG2 = new AutoSector("SG2", "SC");
  private AutoSector SG3 = new AutoSector("SG3", "SC");
  private AutoSector CG1 = new AutoSector("CG1", "SC");
  private AutoSector CG2 = new AutoSector("CG2", "SC");
  private AutoSector CG3 = new AutoSector("CG3", "SC"); 
  private AutoSector CG4 = new AutoSector("CG4", "SC");
  private AutoSector CG5 = new AutoSector("CG5", "SC");
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
    Autos.add(Speaker3());
    Autos.add(EightGP());
    Autos.add(straight());
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
    paths.add(new AutoSector("TestingPath", "SC"));

    AutoTracker tracker = new AutoTracker(subsystems, paths, () -> PositionConstants.Speaker());
    return new Auto("straight", tracker.asCommand(), PositionConstants.Speaker());
  }

  public Auto Speaker3() {
    List<AutoSector> paths = new ArrayList<>();
    paths.add(SG1);
    paths.add(SG2);
    paths.add(SG3);
    AutoTracker tracker = new AutoTracker(subsystems, paths, () -> PositionConstants.Speaker());

    return new Auto("Speaker3", tracker.asCommand(), PositionConstants.Speaker() );
  }
  public Auto EightGP() {
    List<AutoSector> paths = new ArrayList<>();
    paths.add(SG1);
    paths.add(SG2);
    paths.add(SG3);
    paths.add(CG1);
    paths.add(CG2);
    paths.add(CG3);
    paths.add(CG4);
    paths.add(CG5);
    
    AutoTracker tracker = new AutoTracker(subsystems, paths, () -> PositionConstants.Speaker());
    return new Auto("8GamePiece", tracker.asCommand(), PositionConstants.Speaker() );
  }

}