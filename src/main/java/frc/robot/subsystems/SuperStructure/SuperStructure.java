package frc.robot.subsystems.SuperStructure;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorCommands;


public class SuperStructure {
    private ArmSubsystem Arm;
    private ElevatorSubsystem Elev;
    public SuperStructure(ArmSubsystem Arm, ElevatorSubsystem Elev) {
        this.Arm = Arm;
        this.Elev = Elev;
    } 

    public Command calculateArm(Command cmd, Distance curPivotPointDistance) {
        if (!willArmColide(curPivotPointDistance)) {
            return cmd;
        } else {
            return new InstantCommand();
        }
        
    }
    public boolean willArmColide(Distance heightSetpoint) {
        heightSetpoint = heightSetpoint.minus(GeneralConstants.bellypanToGround);
        Distance armLength = ArmConstants.armLength;
        if (armLength.gt(heightSetpoint)) {
            return true;
        }
        return false;
    }
    // Overloaded method with also angle
    public boolean willArmCollide(Distance heightSetpoint, Angle armAngle) {
        Distance distanceFromGround = ArmConstants.armLength.times(Math.cos(armAngle.in(Radians)));
        // TODO: do
        // if (distanceFromGround < Elev.armPivotheight) {
        //     return true;
        // }
        return false;
    }
     
    public Command calculate(Command armSet, Command elevSet) {
        return Commands.parallel(calculateArm(armSet, null/*Elev.armPivotheight()*/), elevSet);
    }
    public class SuperStructureCommands{
        
        public Command L1() {
            return Commands.run(() -> calculate(Arm.Commands.L1(), /*Elev.elevatorCommands.L1()*/));
        }
        public Command L2(){
            return null;
        }
        public Command L3(){
            return L2();
        }
        public Command L4(){
            return null;
        }
        public Command Intake(){
            return null;
        }
    }
}