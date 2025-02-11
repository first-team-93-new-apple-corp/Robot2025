package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Utilities.CommandLimitSwitch;

public class ElevatorSubsystem extends SubsystemBase {
    // Kraken x60 (Controlled by TalonFX) 12-1 gear
    // 18-toothed sprocket 1-1 chain
    // 32.9 in max extension

    private TalonFX outerElevatorMotor;
    private TalonFX innerElevatorMotor;

    private MotionMagicVoltage outerElevatorMotorMagic = new MotionMagicVoltage(0);
    private MotionMagicVoltage innerElevatorMotorMagic = new MotionMagicVoltage(0);

    public ElevatorCommands elevatorCommands = new ElevatorCommands();

    private TalonFXConfiguration outerConfig;
    private TalonFXConfiguration innerConfig;

    private CommandLimitSwitch InnerTopSwitch = new CommandLimitSwitch(0);
    private CommandLimitSwitch InnerBottomSwitch = new CommandLimitSwitch(0);
    private CommandLimitSwitch OuterTopSwitch = new CommandLimitSwitch(0);
    private CommandLimitSwitch OuterBottomSwitch = new CommandLimitSwitch(0);

    // public static final ElevatorSim m_elevatorSim =
    // new ElevatorSim(DCMotor.getKrakenX60(1),
    // ElevatorConstants.kElevatorGearing,
    // ElevatorConstants.kCarriageMass,
    // ElevatorConstants.kElevatorDrumRadius,
    // ElevatorConstants.kMinElevatorHeightMeters,
    // ElevatorConstants.kMaxElevatorHeightMeters,
    // true,
    // 0,
    // 0.01,
    // 0.0);

    public ElevatorSubsystem() {
        outerElevatorMotor = new TalonFX(ElevatorConstants.outerElevatorMotorID);
        innerElevatorMotor = new TalonFX(ElevatorConstants.innerElevatorMotorID);
        var outerSlot0 = outerConfig.Slot0;
        outerSlot0.kP = 1;
        outerSlot0.kD = 1;
        outerSlot0.kG = 0.12;
        outerSlot0.kA = 0.01;
        outerSlot0.kV = 3.11;
        var innerSlot0 = innerConfig.Slot0;
        innerSlot0.kP = 1;
        innerSlot0.kD = 1;
        innerSlot0.kG = 0.16;
        innerSlot0.kA = 0.02;
        innerSlot0.kV = 3.11;
        outerElevatorMotor.getConfigurator().apply(outerConfig);
        innerElevatorMotor.getConfigurator().apply(innerConfig);
        OuterBottomSwitch.Tripped().onTrue(elevatorCommands.zeroOuterMotor());
        InnerBottomSwitch.Tripped().onTrue(elevatorCommands.zeroInnerMotor());
    }

    private Angle DistanceToAngle(Distance D, double Ratio) {
        return Rotations.of(D.in(Inches) * Ratio);
    }

    @Override
    public void simulationPeriodic() {
        System.out.println(outerElevatorMotorMagic.Position);
        // m_elevatorSim.setInput(elevatorMotor.getMotorVoltage().getValueAsDouble() *
        // RobotController.getBatteryVoltage());
        // m_elevatorSim.update(0.020);

        // RoboRioSim.setVInVoltage(
        // BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

    }

    public class ElevatorCommands {
        public Command L1() {

            return runOnce(() -> {
                outerElevatorMotor.setControl(outerElevatorMotorMagic.withPosition(
                        DistanceToAngle(ElevatorConstants.outerL1Setpoint, ElevatorConstants.OuterRotationsToInches)));
                innerElevatorMotor.setControl(innerElevatorMotorMagic.withPosition(
                        DistanceToAngle(ElevatorConstants.innerL1Setpoint, ElevatorConstants.InnerRotationsToInches)));

            });
            // TODO remove if not needed
            // return runOnce(() -> setSpeed(1));

        }

        public Command L2() {
            return runOnce(() -> {
                outerElevatorMotor.setControl(outerElevatorMotorMagic.withPosition(
                        DistanceToAngle(ElevatorConstants.outerL2Setpoint, ElevatorConstants.OuterRotationsToInches)));
                innerElevatorMotor.setControl(innerElevatorMotorMagic.withPosition(
                        DistanceToAngle(ElevatorConstants.innerL2Setpoint, ElevatorConstants.InnerRotationsToInches)));

            });
        }

        public Command L3() {
            return runOnce(() -> {
                outerElevatorMotor.setControl(outerElevatorMotorMagic.withPosition(
                        DistanceToAngle(ElevatorConstants.outerL3Setpoint, ElevatorConstants.OuterRotationsToInches)));
                innerElevatorMotor.setControl(innerElevatorMotorMagic.withPosition(
                        DistanceToAngle(ElevatorConstants.innerL3Setpoint, ElevatorConstants.InnerRotationsToInches)));

            });
        }

        public Command L4() {
            return runOnce(() -> {
                outerElevatorMotor.setControl(outerElevatorMotorMagic.withPosition(
                        DistanceToAngle(ElevatorConstants.outerL4Setpoint, ElevatorConstants.OuterRotationsToInches)));
                innerElevatorMotor.setControl(innerElevatorMotorMagic.withPosition(
                        DistanceToAngle(ElevatorConstants.innerL4Setpoint, ElevatorConstants.InnerRotationsToInches)));

            });
        }

        public Command zeroHeight() {
            return Commands.print("Zeroed I guess");

        }

        public Command zeroOuterMotor() {
            return runOnce(() -> {
                outerElevatorMotor.setControl(outerElevatorMotorMagic.withPosition(0));
            });
        }

        public Command zeroInnerMotor() {
            return runOnce(() -> {
                outerElevatorMotor.setControl(outerElevatorMotorMagic.withPosition(0));
            });
        }

    }
}
