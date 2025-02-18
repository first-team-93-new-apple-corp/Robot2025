package frc.robot.subsystems.Arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.commands.intake;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

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
    private double Intake, L1, L2, L4, Setpoint;

    public ArmSubsystem() {
        Commands = new ArmCommands();
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

        // Setpoints
        Setpoint = 0;
        L1 = 90; // TODO find values
        L2 = 113;
        L4 = 113; //
        Intake = 45;
        // Intake = -90.0; // -90 degree from ground
        m_Encoder.setInverted(true);
    }

    public double getPosition() {
        return m_Encoder.get() * 180;
    }

    public double getSetpoint() {
        return Setpoint;
    }

    public boolean atSetpoint() {
        return Math.abs(getPosition() - Setpoint) < 1;
    }

    public void runAngle(double angle) {
        wrist.setControl(mmVolt.withPosition(angle));
    }

    @Override
    public void periodic() {
        wrist.setPosition(getPosition());

        SmartDashboard.putNumber("WristAngleMotor", wrist.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("WristAngleEncoder", getPosition());
    }

    public class ArmCommands {
        public Command L1() {
            Setpoint = L1;
            return run(() -> runAngle(Setpoint));
        }

        public Command L2() {
            Setpoint = L2;
            return run(() -> runAngle(Setpoint));
        }

        public Command L3() {
            return L2();
        }

        public Command L4() {
            Setpoint = L4;
            return run(() -> runAngle(Setpoint));
        }

        public Command Intake() {
            Setpoint = Intake;
            return run(() -> runAngle(Setpoint));
        }
    }
}
