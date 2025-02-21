package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorStrategy;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Grabber.GrabberSubsystem;

public class intake extends SequentialCommandGroup {
    ElevatorSubsystem m_Elev;
    ArmSubsystem m_Arm;
    GrabberSubsystem m_Grab;

    public intake(ElevatorSubsystem m_Elev, ArmSubsystem m_Arm, GrabberSubsystem m_Grab) {
        this.m_Elev = m_Elev;
        this.m_Arm = m_Arm;
        this.m_Grab = m_Grab;
        addCommands(m_Elev.Commands.intake().until(() -> m_Elev.atSetpoint()));
        addCommands(m_Arm.Commands.Intake().until(() -> m_Arm.atSetpoint()));
        addCommands(m_Grab.Commands.intake().alongWith(m_Elev.Commands.changeSetpointBy(Inches.of(-3), ElevatorStrategy.stageOneBias)).until(() -> m_Elev.atSetpoint()));
        addCommands(m_Grab.Commands.intake().alongWith(m_Elev.Commands.changeSetpointBy(Inches.of(3), ElevatorStrategy.stageOneBias)).until(() -> m_Elev.atSetpoint()));
        addCommands(m_Arm.Commands.L2());
    }
}
