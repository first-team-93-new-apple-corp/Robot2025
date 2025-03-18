package frc.robot.subsystems.Grabber;


import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.GrabberConstants;
import frc.robot.Utilities.CommandLimitSwitch;
import frc.robot.Utilities.CommandLimitSwitchDio;
import frc.robot.commands.intake;

public class GrabberSubsystem implements Subsystem {
    SparkMax motor;
    public GrabberCommands Commands = new GrabberCommands();
    public static CommandLimitSwitchDio limit;

    public GrabberSubsystem() {
        motor = new SparkMax(GrabberConstants.Grabber, MotorType.kBrushless);
        limit = new CommandLimitSwitchDio(Constants.GrabberConstants.LimitSwitch);
    }

    @Override
    public void periodic() {
        hasCoral();
    }

    public boolean hasCoral(){
        return limit.triggered();
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
        public BooleanSupplier checkCoral(){
            return () -> hasCoral();
        }
    }

}
