package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{

    private SparkMax motor;
    public IntakeCommands Commands = new IntakeCommands();
    public IntakeSubsystem() {
        motor = new SparkMax(61, MotorType.kBrushless);
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public class IntakeCommands {
        public Command stop() {
            return runOnce(() -> setSpeed(0));
        }

        public Command intake() {
            return runOnce(() -> setSpeed(-0.4));
        }

        public Command outtake() {
            return runOnce(() -> setSpeed(0.4));
        }

        public Command setSpeed(double speed) {
            return runOnce(() -> setSpeed(-speed));
        }
    }
}
