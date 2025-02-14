package frc.robot.subsystems.Grabber;

import frc.robot.Constants.GrabberConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class GrabberSubsystem implements Subsystem {
    SparkMax motor;
    public GrabberCommands Commands = new GrabberCommands();
    private DigitalInput grabberLimit;

    public GrabberSubsystem() {
        motor = new SparkMax(GrabberConstants.Grabber, MotorType.kBrushless);
        grabberLimit = new DigitalInput(9);
        // GrabberSubsystem Constructor
    }

    @Override
    public void periodic() {
        // GrabberSubsystem periodic
    }

    public boolean hasCoral(){
        return grabberLimit.get();
    }

    public void setSpeed(double speed) {
        motor.set(speed);
        if (speed == 0)
            motor.stopMotor();
    }

    public class GrabberCommands {
        public Command intake() {
            return runOnce(() -> setSpeed(GrabberConstants.intakeSpeed)).until(() -> hasCoral()).andThen(() -> setSpeed(0)); 
        }

        public Command outtake() {
            return runOnce(() -> setSpeed(GrabberConstants.outakeSpeed));
        }

        public Command brake() {
            return runOnce(() -> setSpeed(0));
        }
    }

}
