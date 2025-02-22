package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorStrategy;
import frc.robot.Utilities.CommandLimitSwitchDio;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Grabber.GrabberSubsystem;

public class intake extends SequentialCommandGroup {
    ElevatorSubsystem m_Elev;
    ArmSubsystem m_Arm;
    GrabberSubsystem m_Grab;
    CommandLimitSwitchDio m_IntakeLimit = new CommandLimitSwitchDio(Constants.GrabberConstants.LimitSwitch);

    public intake(ElevatorSubsystem m_Elev, ArmSubsystem m_Arm, GrabberSubsystem m_Grab) {
        this.m_Elev = m_Elev;
        this.m_Arm = m_Arm;
        this.m_Grab = m_Grab;

        addCommands(m_Elev.Commands.intake().alongWith(Commands.waitUntil(() -> m_Elev.atSetpoint())));
        addCommands(Commands.print("Elevator has ended"));

        addCommands(m_Arm.Commands.Intake().alongWith(Commands.waitUntil(() -> m_Arm.atSetpoint())));
        addCommands(Commands.print("Arm has ended"));
        addCommands((m_Grab.Commands.intake().withDeadline(Commands.waitSeconds(1)))
                .alongWith(m_Elev.Commands.changeSetpointBy(Inches.of(-4.75), ElevatorStrategy.stageOneBias)
                        .alongWith(Commands.waitUntil(() -> m_Elev.atSetpoint()))));
        addCommands(Commands.print("Intake has ended"));

        addCommands(
                (m_Grab.Commands.intake()
                        .alongWith(m_Elev.Commands.changeSetpointBy(Inches.of(4.75), ElevatorStrategy.stageOneBias))
                        .alongWith(Commands.waitUntil(() -> m_Elev.atSetpoint())))
                        .until(() -> m_IntakeLimit.triggered()));

        addCommands(m_Elev.Commands.L2().alongWith(Commands.waitUntil(() -> m_Elev.atSetpoint())));
        addCommands(m_Arm.Commands.L2());
    }
}
