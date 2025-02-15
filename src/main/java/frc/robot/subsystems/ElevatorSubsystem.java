package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    public ElevatorCommands Commands = new ElevatorCommands();

    private TalonFXConfiguration outerConfig = new TalonFXConfiguration();
    private TalonFXConfiguration innerConfig = new TalonFXConfiguration();

    private CommandLimitSwitch InnerTopSwitch = new CommandLimitSwitch(ElevatorConstants.InnerTopChannel);
    private CommandLimitSwitch InnerBottomSwitch = new CommandLimitSwitch(ElevatorConstants.InnerBottomChannel);
    private CommandLimitSwitch OuterTopSwitch = new CommandLimitSwitch(ElevatorConstants.OuterTopChannel);
    private CommandLimitSwitch OuterBottomSwitch = new CommandLimitSwitch(ElevatorConstants.OuterBottomChannel);

    private Distance elevatorSetpoint = Inches.of(0);

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
    MotionMagicConfigs outerMMConfig;
    MotionMagicConfigs innerMMConfig;

    public ElevatorSubsystem() {
        outerElevatorMotor = new TalonFX(ElevatorConstants.outerElevatorMotorID, "rio");
        innerElevatorMotor = new TalonFX(ElevatorConstants.innerElevatorMotorID, "Drivetrain");

        outerMMConfig = new MotionMagicConfigs();
        outerMMConfig = outerConfig.MotionMagic;
        outerMMConfig.MotionMagicCruiseVelocity = 80;
        outerMMConfig.MotionMagicAcceleration = 160;

        var outerSlot0 = outerConfig.Slot0;
        outerSlot0.kP = 1;
        outerSlot0.kD = 1;
        outerSlot0.kG = 0.34;
        outerSlot0.kA = 0.04;
        outerSlot0.kV = 3.01;

        var innerSlot0 = innerConfig.Slot0;
        innerSlot0.kP = 1;
        innerSlot0.kD = 1;
        innerSlot0.kG = 0.16;
        innerSlot0.kA = 0.02;
        innerSlot0.kV = 3.11;

        innerMMConfig = new MotionMagicConfigs();
        innerMMConfig = outerConfig.MotionMagic;
        innerMMConfig.MotionMagicCruiseVelocity = 80;
        innerMMConfig.MotionMagicAcceleration = 160;

        outerElevatorMotor.getConfigurator().apply(outerConfig);
        innerElevatorMotor.getConfigurator().apply(innerConfig);

        OuterBottomSwitch.Tripped().onTrue(Commands.zeroOuterMotor());
        InnerBottomSwitch.Tripped().onTrue(Commands.zeroInnerMotor());
    }

    private Distance armPivotHeight() {
        return Inches
                .of(outerElevatorMotor.getPosition().getValue().timesRatio(ElevatorConstants.OuterRotationsToInches)
                        .in(Inches))
                .plus(innerElevatorMotor.getPosition().getValue().timesRatio(ElevatorConstants.InnerRotationsToInches));
    }

    private void setSetpoints(Distance D) {
        elevatorSetpoint = D;
        Distance outer = D.times(.54);
        Distance inner = D.times(.45);

        outerElevatorMotor.setControl(outerElevatorMotorMagic
                .withPosition((outer.divideRatio(ElevatorConstants.OuterRotationsToInches).in(Rotations))));
        innerElevatorMotor.setControl(innerElevatorMotorMagic
                .withPosition((inner.divideRatio(ElevatorConstants.InnerRotationsToInches).in(Rotations))));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevtor Height", armPivotHeight().in(Inches));
        SmartDashboard.putNumber("Elevtor Height Setpoint", elevatorSetpoint.in(Inches));

        // m_elevatorSim.setInput(elevatorMotor.getMotorVoltage().getValueAsDouble() *
        // RobotController.getBatteryVoltage());
        // m_elevatorSim.update(0.020);

        // RoboRioSim.setVInVoltage(
        // BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    }

    public class ElevatorCommands {
        public Command L1() {
            return runOnce(() -> {
                setSetpoints(ElevatorConstants.L1Setpoint);

            });
        }

        public Command L2() {
            return runOnce(() -> {
                setSetpoints(ElevatorConstants.L2Setpoint);

            });
        }

        public Command L3() {
            return runOnce(() -> {
                setSetpoints(ElevatorConstants.L3Setpoint);

            });
        }

        public Command L4() {
            return runOnce(() -> {
                setSetpoints(ElevatorConstants.L4Setpoint);

            });
        }

        public Command zeroOuterMotor() {
            return runOnce(() -> {
                outerElevatorMotor.setPosition(0);
            }).ignoringDisable(true);
        }

        public Command zeroInnerMotor() {
            return runOnce(() -> {
                innerElevatorMotor.setPosition(0);
            }).ignoringDisable(true);
        }

    }
}
