package frc.robot.subsystems.SuperStructure;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Grabber.GrabberSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.Elevator_Movements;
import frc.robot.Constants.ElevatorConstants.Elevator_Positions;
import frc.robot.Constants.ElevatorConstants.Elevator_Stages;
import frc.robot.Constants.GeneralConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class SuperStructure {
    private ArmSubsystem Arm;
    private ElevatorSubsystem Elev;

    private Elevator_Stages movements[];
    private Elevator_Stages currentStage;
    private Elevator_Positions desiredPosition;
    private int eventIndex;
    public SuperStructureCommands commands = new SuperStructureCommands();

    public SuperStructure(ArmSubsystem Arm, ElevatorSubsystem Elev, GrabberSubsystem Grab) {
        this.Arm = Arm;
        this.Elev = Elev;
        currentStage = Elevator_Stages.End;
        desiredPosition = Elevator_Positions.Intake;
        eventIndex = 0;
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
            return false;
        }
        return true;
    }

    // Overloaded method with also angle
    public boolean willArmCollide(Distance heightSetpoint, Angle armAngle) {
        Distance armHeight = ArmConstants.armLength.times(Math.cos(armAngle.in(Radians)));
        // TODO: do
        if (armHeight.gt(ElevatorConstants.elevatorHeight)) {
            return true;
        }
        return false;
    }

    private Elevator_Positions getPosition() {
        if (Elev.getPosition() != Elevator_Positions.L1 && Elev.getPosition() != Elevator_Positions.Intake) {
            return Elev.getPosition();
        } else {
            if (Arm.getAngle().lt(Degrees.of(5))) {
                return Elevator_Positions.Intake;
            }
            return Elevator_Positions.L1;
        }
    }

    public Elevator_Movements getMovement(Elevator_Positions setpoint) {
        switch (getPosition()) {
            case Intake:
                switch (setpoint) {
                    case Intake:
                        return Elevator_Movements.Intake_To_Intake;
                    case L1:
                        return Elevator_Movements.Intake_To_L1;
                    case L2:
                        return Elevator_Movements.Intake_To_L2;
                    case L3:
                        return Elevator_Movements.Intake_To_L3;
                    case L4:
                        return Elevator_Movements.Intake_To_L4;
                }
            case L1:
                switch (setpoint) {
                    case L1:
                        return Elevator_Movements.L1_To_L1;
                    case L2:
                        return Elevator_Movements.L1_To_L2;
                    case L3:
                        return Elevator_Movements.L1_To_L3;
                    case L4:
                        return Elevator_Movements.L1_To_L4;
                    case Intake:
                        return Elevator_Movements.L1_To_Intake;
                }
            case L2:
                switch (setpoint) {
                    case L1:
                        return Elevator_Movements.L2_To_L1;
                    case L2:
                        return Elevator_Movements.L2_To_L2;
                    case L3:
                        return Elevator_Movements.L2_To_L3;
                    case L4:
                        return Elevator_Movements.L2_To_L4;
                    case Intake:
                        return Elevator_Movements.L2_To_Intake;
                }
            case L3:
                switch (setpoint) {
                    case L1:
                        return Elevator_Movements.L3_To_L1;
                    case L2:
                        return Elevator_Movements.L3_To_L2;
                    case L3:
                        return Elevator_Movements.L3_To_L3;
                    case L4:
                        return Elevator_Movements.L3_To_L4;
                    case Intake:
                        return Elevator_Movements.L3_To_Intake;
                }
            case L4:
                switch (setpoint) {
                    case L1:
                        return Elevator_Movements.L4_To_L1;
                    case L2:
                        return Elevator_Movements.L4_To_L2;
                    case L3:
                        return Elevator_Movements.L4_To_L3;
                    case L4:
                        return Elevator_Movements.L4_To_L4;
                    case Intake:
                        return Elevator_Movements.L4_To_Intake;
                }
            default:
                return Elevator_Movements.Do_Nothing;
        }
    }

    private void allToSetpoint() {
        elevatorToSetpoint();
        armToSetpoint();
    }

    private void elevatorToSetpoint() {
        switch (desiredPosition) {
            case Intake:
                Elev.Commands.Intake();
            case L1:
                Elev.Commands.L1();
            case L2:
                Elev.Commands.L2();
            case L3:
                Elev.Commands.L3();
            case L4:
                Elev.Commands.L4();
        }

    }

    private void armToSetpoint() {
        switch (desiredPosition) {
            case Intake:
                Arm.Commands.Stow();
            case L1:
                Arm.Commands.L1();
            case L2:
                Arm.Commands.L2();
            case L3:
                Arm.Commands.L3();
            case L4:
                Arm.Commands.L4();
        }
    }

    public void resetSubsystem(Elevator_Positions setpoint) {
        desiredPosition = setpoint;

        eventIndex = 0;
        movements = getMovement(desiredPosition).getEvents();

        currentStage = movements[eventIndex];
    }

    public void follow() {
        switch (currentStage) {
            case All:
                allToSetpoint();
                break;
            case Elevator:
                elevatorToSetpoint();
                break;
            case Arm:
                armToSetpoint();
                break;

            case End:
                allToSetpoint();
                break;
            default:
                
                break;
        }
    }
    public void continueToSetpoint() {
        if (Elev.atSetpoint(desiredPosition) && Arm.atSetpoint(desiredPosition) && eventIndex < movements.length - 1) {
            switchEvent();
            follow();
        }
    }

    public void switchEvent() {
        eventIndex++;
        currentStage = movements[eventIndex];
    }

    public class SuperStructureCommands {

        public Command L1() {
            return Commands.runOnce(() -> resetSubsystem(Elevator_Positions.L1));
        }

        public Command L2() {
            return Commands.runOnce(() -> resetSubsystem(Elevator_Positions.L2));
        }

        public Command L3() {
            return Commands.runOnce(() -> resetSubsystem(Elevator_Positions.L3));
        }

        public Command L4() {
            return Commands.runOnce(() -> resetSubsystem(Elevator_Positions.L4));
        }

        public Command Intake() {
            return Commands.runOnce(() -> resetSubsystem(Elevator_Positions.Intake));
        }

        public Command continueToSetpoint() {
            return Commands.runOnce(() -> continueToSetpoint());
        }
    }
}