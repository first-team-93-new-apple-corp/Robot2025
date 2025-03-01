package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorStrategy;
import frc.robot.Utilities.CommandLimitSwitch;
import frc.robot.Utilities.CommandLimitSwitchDio;

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
        innerElevatorMotor = new TalonFX(ElevatorConstants.innerElevatorMotorID, "DriveTrain");

        outerMMConfig = new MotionMagicConfigs();
        outerMMConfig = outerConfig.MotionMagic;
        outerMMConfig.MotionMagicCruiseVelocity = 80;
        outerMMConfig.MotionMagicAcceleration = 160;

        var outerSlot0 = outerConfig.Slot0;
        outerSlot0.kP = 3.5;
        outerSlot0.kD = 0.03;
        outerSlot0.kG = 0.6;
        outerSlot0.kA = 0;
        outerSlot0.kV = 0.15;
        outerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        var innerSlot0 = innerConfig.Slot0;
        innerSlot0.kP = 1;
        innerSlot0.kD = 0;
        innerSlot0.kG = 0.44;
        innerSlot0.kA = 0;
        innerSlot0.kV = .14;

        innerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        innerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        innerMMConfig = new MotionMagicConfigs();
        innerMMConfig = innerConfig.MotionMagic;
        innerMMConfig.MotionMagicCruiseVelocity = 80;
        innerMMConfig.MotionMagicAcceleration = 160;

        outerElevatorMotor.getConfigurator().apply(outerConfig);
        innerElevatorMotor.getConfigurator().apply(innerConfig);

        // OuterBottomSwitch.Tripped().onTrue(Commands.zeroOuterMotor());
        // InnerBottomSwitch.Tripped().onTrue(Commands.zeroInnerMotor());
        SmartDashboard.putData("zero carriage", Commands.zeroInnerMotor());
        SmartDashboard.putData("zero ele", Commands.zeroOuterMotor());
        SmartDashboard.putData("Coast carriage", Commands.brakeInnerMotor(false));
        SmartDashboard.putData("Coast ele", Commands.brakeOuterMotor(false));
        SmartDashboard.putData("Brake carriage", Commands.brakeInnerMotor(true));
        SmartDashboard.putData("Brake ele", Commands.brakeOuterMotor(true));
        // OuterTopSwitch.Tripped().onTrue(Commands.maxOuterMotor());
        // InnerTopSwitch.Tripped().onTrue(Commands.maxInnerMotor());
    }

    private Distance armPivotHeight() {
        return Inches
                .of(outerElevatorMotor.getPosition().getValue().timesRatio(ElevatorConstants.OuterRotationsToInches)
                        .in(Inches))
                .plus(innerElevatorMotor.getPosition().getValue().timesRatio(ElevatorConstants.InnerRotationsToInches));
    }

    private void setSetpoints(Distance D) {
        setSetpoints(D, ElevatorStrategy.noBias);
    }

    private void setSetpoints(Distance D, ElevatorStrategy Strategy) {
        if (D.gt(Centimeters.of(173))) {
            D = Centimeters.of(173);
        } else if (D.lt((ElevatorConstants.Bottom))) {
            D = ElevatorConstants.Bottom;
        }
        elevatorSetpoint = D;
        Distance outer = elevatorSetpoint.times(.54);
        Distance inner = elevatorSetpoint.times(.45);
        switch (Strategy) {
            case carriageBias:
                if (elevatorSetpoint.gt(Inches.of(31))) {
                    inner = Inches.of(31);
                    outer = elevatorSetpoint.minus(inner).times(0.98);
                } else {
                    inner = elevatorSetpoint.times(1);
                    outer = Inches.of(0.2);
                }
                break;
            case noBias:
                outer = elevatorSetpoint.times(.54);
                inner = elevatorSetpoint.times(.45);
                break;
            case stageOneBias:
                if (elevatorSetpoint.gt(Inches.of(36.5))) {
                    outer = Inches.of(36.5);
                    inner = elevatorSetpoint.minus(outer).times(.98);
                } else {
                    outer = elevatorSetpoint.times(1);
                    inner = Inches.of(0.5);
                }
                break;
        }

        outerElevatorMotor.setControl(outerElevatorMotorMagic
                .withPosition((outer.divideRatio(ElevatorConstants.OuterRotationsToInches).in(Rotations))));
        innerElevatorMotor.setControl(innerElevatorMotorMagic
                .withPosition((inner.divideRatio(ElevatorConstants.InnerRotationsToInches).in(Rotations))));
    }

    public boolean atSetpoint() {
        return armPivotHeight().isNear(elevatorSetpoint, Centimeters.of(3));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevtor Height", armPivotHeight().in(Inches));
        SmartDashboard.putNumber("Elevtor Height Setpoint", elevatorSetpoint.in(Inches));

        SmartDashboard.putBoolean("Carraige Top", InnerTopSwitch.triggered());
        SmartDashboard.putBoolean("Elevator Top", OuterTopSwitch.triggered());
        SmartDashboard.putBoolean("Carraige Bottom", InnerBottomSwitch.triggered());
        SmartDashboard.putBoolean("Elevator Bottom", OuterBottomSwitch.triggered());
        InnerTopSwitch.publishValue();
        InnerBottomSwitch.publishValue();
        OuterTopSwitch.publishValue();
        OuterBottomSwitch.publishValue();

        // m_elevatorSim.setInput(elevatorMotor.getMotorVoltage().getValueAsDouble() *
        // RobotController.getBatteryVoltage());
        // m_elevatorSim.update(0.020);

        // RoboRioSim.setVInVoltage(
        // BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    }

    public class ElevatorCommands {

        public Command outtake() {
            return runOnce(() -> {
                setSetpoints(elevatorSetpoint.minus(Centimeters.of(20)));

            });

        }

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

        public Command L2(ElevatorStrategy strategy) {
            return runOnce(() -> {
                setSetpoints(ElevatorConstants.L2Setpoint, strategy);

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

        public Command Algea1() {
            return runOnce(() -> {
                setSetpoints(ElevatorConstants.Algea1);

            });
        }

        public Command Algea2() {
            return runOnce(() -> {
                setSetpoints(ElevatorConstants.Algea2);

            });
        }

        public Command Bottom() {
            return runOnce(() -> {
                setSetpoints(ElevatorConstants.Bottom);

            });
        }

        public Command intakePrime() {
            return runOnce(() -> {
                setSetpoints(Inches.of(20.5), ElevatorStrategy.stageOneBias);

            });
        }

        public Command intake() {
            return runOnce(() -> {
                setSetpoints(ElevatorConstants.L2Setpoint, ElevatorStrategy.stageOneBias);

            });
        }

        public Command changeSetpointBy(Distance D) {
            return runOnce(() -> {
                setSetpoints(elevatorSetpoint.plus(D));

            });
        }

        public Command changeSetpointBy(Distance D, ElevatorStrategy strategy) {
            return runOnce(() -> {
                setSetpoints(elevatorSetpoint.plus(D), strategy);

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

        public Command maxInnerMotor() {
            return runOnce(() -> {
                innerElevatorMotor.setPosition(64.5);
            }).ignoringDisable(true);
        }

        public Command maxOuterMotor() {
            return runOnce(() -> {
                innerElevatorMotor.setPosition(97.5);
            }).ignoringDisable(true);
        }

        public Command brakeInnerMotor(boolean brake) {
            return runOnce(() -> {
                if (brake) {
                    innerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                    innerElevatorMotor.getConfigurator().apply(innerConfig);
                } else {
                    innerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                    innerElevatorMotor.getConfigurator().apply(innerConfig);
                }

            }).ignoringDisable(true);
        }

        public Command brakeOuterMotor(boolean brake) {
            return runOnce(() -> {
                if (brake) {
                    outerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                    outerElevatorMotor.getConfigurator().apply(outerConfig);
                } else {
                    outerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                    outerElevatorMotor.getConfigurator().apply(outerConfig);
                }
            }).ignoringDisable(true);
        }

    }
}
