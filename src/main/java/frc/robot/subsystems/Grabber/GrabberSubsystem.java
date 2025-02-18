package frc.robot.subsystems.Grabber;

import frc.robot.Constants.GrabberConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class GrabberSubsystem implements Subsystem {
    SparkMax motor;
    public GrabberCommands Commands = new GrabberCommands();

    public GrabberSubsystem() {
        motor = new SparkMax(GrabberConstants.Grabber, MotorType.kBrushless);
        // GrabberSubsystem Constructor
    }

    @Override
    public void periodic() {
        // GrabberSubsystem periodic
    }

    public void setSpeed(double speed) {
        motor.set(speed);
        if (speed == 0)
            motor.stopMotor();
    }

    public class GrabberCommands {
        public Command intake() {
            return startEnd(() -> setSpeed(GrabberConstants.intakeSpeed), () -> setSpeed(0));
        }

        public Command outtake() {
            return startEnd(() -> setSpeed(GrabberConstants.outakeSpeed), () -> setSpeed(0));
        }

        public Command brake() {
            return runOnce(() -> setSpeed(0));
        }
    }

}
