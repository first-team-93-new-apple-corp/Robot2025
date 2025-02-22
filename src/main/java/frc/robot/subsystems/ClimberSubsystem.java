package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private TalonFX climber;
    private TalonFXConfiguration climberConfig = new TalonFXConfiguration();;

    private DutyCycleEncoder m_Encoder;

    BangBangController controller = new BangBangController();
    public ClimberCommands climberCommands = new ClimberCommands();

    public ClimberSubsystem() {
        climber = new TalonFX(ClimberConstants.climberMotorID, "rio");
        // climberConfig = new TalonFXConfiguration();

        m_Encoder = new DutyCycleEncoder(ClimberConstants.climberEncoderID);

        // PID configuation just in case
        // var slot0 = climberConfig.Slot0;
        // slot0.kA = 0.0;
        // slot0.kG = 0.0;
        // slot0.kV = 0.0;
        // slot0.kP = 0.0;
        // slot0.kI = 0.0;
        // slot0.kD = 0.0;
        // slot0.kS = 0.0;

        climber.getConfigurator().apply(climberConfig);

    }

    private void runAngle(Double angle) {

        climber.set(
                controller.calculate((m_Encoder.get() * 360) - ClimberConstants.encoderOffset, angle));
    }

    public class ClimberCommands {
        public Command inwardPosition() {
            return run(() -> runAngle(ClimberConstants.inSetpoint));
        }

        public Command outwardPosition() {
            return run(() -> runAngle(ClimberConstants.outSetpoint));
        }

        public Command stop() {
            return run(() -> climber.set(0));
        }
    }

}
