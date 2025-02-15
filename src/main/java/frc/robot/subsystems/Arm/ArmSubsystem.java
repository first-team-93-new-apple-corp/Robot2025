package frc.robot.subsystems.Arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants.Elevator_Positions;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private TalonFX wrist;
    private TalonFXConfiguration wristConfig;

    private MotionMagicVoltage mmVolt;
    private MotionMagicConfigs mmConfig;

    private DutyCycleEncoder m_Encoder;
    public ArmCommands Commands;
    // Setpoints
    private double Intake, IntakeOffset;
    // Limts
    private double lowLimit, highLimit;

    public ArmSubsystem() {
        Commands = new ArmCommands();
        m_Encoder = new DutyCycleEncoder(ArmConstants.IDs.Encoder);
        wrist = new TalonFX(ArmConstants.IDs.Wrist);
        m_Encoder = new DutyCycleEncoder(ArmConstants.IDs.Encoder);
        wrist = new TalonFX(ArmConstants.IDs.Wrist);
        wristConfig = new TalonFXConfiguration();
        mmVolt = new MotionMagicVoltage(0);
        mmConfig = new MotionMagicConfigs();
        mmConfig = wristConfig.MotionMagic;
        mmConfig.MotionMagicCruiseVelocity = 80;
        mmConfig.MotionMagicAcceleration = 160;

        var slot0 = wristConfig.Slot0;
        slot0.kA = 0.0; // TODO find values
        slot0.kG = -0.07;
        slot0.kV = 0.1;
        slot0.kP = 0.3; // TODO tune values
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        slot0.kS = 0.0;
        wrist.getConfigurator().apply(wristConfig);

        // Limits
        lowLimit = 0.0;
        highLimit = 0.0;
        // Setpoints
        L1 = -45.0; // TODO find values
        L2 = -20.0;
        L3 = L2; //
        L4 = -45.0; //
        Intake = -90.0; // -90 degree from ground

        Intake = 0.0; // -90 degree from ground
        IntakeOffset = 0.0;
        
    }

    public double getPosition() {
        return m_Encoder.get();
    }
    public Angle getAngle() {
        return Degrees.of(getPosition()*360 - IntakeOffset);
    }
    public void runAngle(Angle angle) {
        wrist.setControl(mmVolt.withPosition(angle));
    }

    public boolean atSetpoint(Elevator_Positions setpoint) {
        switch (setpoint) {
            case Intake:
                return getAngle().isNear(ArmConstants.Setpoints.Intake, Degrees.of(5));
            case L1:
                return getAngle().isNear(ArmConstants.Setpoints.L1, Degrees.of(5));
            case L2:
                return getAngle().isNear(ArmConstants.Setpoints.L2, Degrees.of(5));
            case L3:
                return getAngle().isNear(ArmConstants.Setpoints.L3, Degrees.of(5));
            case L4:
                return getAngle().isNear(ArmConstants.Setpoints.L4, Degrees.of(5));
            default:
                return false;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("WristAngle", m_Encoder.getFrequency());
    }

    public class ArmCommands {
        public Command L1() {
            return run(() -> runAngle(ArmConstants.Setpoints.L1));
        }

        public Command L2() {
            return run(() -> runAngle(ArmConstants.Setpoints.L2));
        }

        public Command L3() {
            return run(() -> runAngle(ArmConstants.Setpoints.L3));
        }

        public Command L4() {
            return run(() -> runAngle(ArmConstants.Setpoints.L4));
        }

        public Command Stow() {
            return run(() -> runAngle(ArmConstants.Setpoints.Intake));
        }
    }
}
