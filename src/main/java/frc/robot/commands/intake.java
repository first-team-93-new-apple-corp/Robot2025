package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Grabber.GrabberSubsystem;

public class intake extends SequentialCommandGroup{
    ElevatorSubsystem m_Elev;
    ArmSubsystem m_Arm;
    GrabberSubsystem m_Grab;
    public intake(ElevatorSubsystem m_Elev, ArmSubsystem m_Arm, GrabberSubsystem m_Grab){
        this.m_Elev = m_Elev;
        this.m_Arm = m_Arm;
        this.m_Grab = m_Grab;
        addCommands(m_Elev.Commands.L3());
        addCommands(m_Arm.Commands.Stow());
        addCommands(m_Grab.Commands.intake().alongWith(m_Elev.Commands.L2()));
        addCommands(m_Elev.Commands.L3());
        addCommands(m_Arm.Commands.L1());

    }
    // public Command intakeFromBelly(){
    //     return intake.
    // }
}
