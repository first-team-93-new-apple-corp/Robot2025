package frc.robot.subsystems.Arm;

import frc.robot.Constants.ArmConstants;

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
    private double Intake, L1, L2, L3, L4;
    // Limts
    private double lowLimit, highLimit;

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
        slot0.kG = 0.0;
        slot0.kV = 0.0;
        slot0.kP = 1.0; // TODO tune values
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        slot0.kS = 0.0;
        wrist.getConfigurator().apply(wristConfig);

        // Limits
        lowLimit = 0.0;
        highLimit = 0.0;
        // Setpoints
        L1 = 0.0; // TODO find values
        L2 = 0.0;
        L3 = L2; //
        L4 = 10.0; //
        Intake = 0.0; // -90 degree from ground
    }

    public double getPosition() {
        return m_Encoder.get();
    }

    public void runAngle(double angle) {
        wrist.setControl(mmVolt.withPosition(angle));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("WristAngle", m_Encoder.getFrequency());
    }

    public class ArmCommands {
        public Command L1() {
            return run(() -> runAngle(L1));
        }

        public Command L2() {
            return run(() -> runAngle(L2));
        }

        public Command L3() {
            return run(() -> runAngle(L3));
        }

        public Command L4() {
            return run(() -> runAngle(L4));
        }

        public Command Stow() {
            return run(() -> runAngle(Intake));
        }
    }
}
