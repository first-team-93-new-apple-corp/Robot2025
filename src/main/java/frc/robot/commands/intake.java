package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

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

        addCommands(m_Elev.Commands.intake().until(() -> m_Elev.atSetpoint()));
        addCommands(m_Arm.Commands.Intake().until(() -> m_Arm.atSetpoint()));
        addCommands(
                m_Grab.Commands.intake()
                        .until(m_IntakeLimit.Tripped())
                        .alongWith(m_Elev.Commands.changeSetpointBy(Inches.of(-1.5), ElevatorStrategy.stageOneBias))
                        .until(() -> m_Elev.atSetpoint()));
        addCommands(
                m_Grab.Commands.intake()
                        .until(m_IntakeLimit.Tripped())
                        .alongWith(m_Elev.Commands.changeSetpointBy(Inches.of(1.5), ElevatorStrategy.stageOneBias))
                        .until(() -> m_Elev.atSetpoint()));
                        
        addCommands(m_Elev.Commands.L2().until(() -> m_Elev.atSetpoint()));
        addCommands(m_Arm.Commands.L2());
    }
}
