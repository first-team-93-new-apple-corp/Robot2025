package frc.robot.subsystems.Controlles;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class POVDriveV2 implements ControllerSchemeIO {

    private CommandJoystick LeftStick;
    private CommandJoystick RightStick;
    private Supplier<Double> robotAngle;
    private double Angle;
    private double PerspectiveOffset;
    private boolean m_hasAppliedOperatorPerspective = false;

    /**
     * An implementation of {@link #the (ControllerSchemeIO)}
     * <p>
     * Uses left stick to generate center of rotation with FeildRel
     */
    public POVDriveV2(int LeftPort, int RightPort, Supplier<Double> robotAngle) {
        LeftStick = new CommandJoystick(LeftPort);
        RightStick = new CommandJoystick(RightPort);
        this.robotAngle = robotAngle;
        LeftStick.button(2).onTrue((new InstantCommand(() -> Angle = this.robotAngle.get())));
        LeftStick.povCenter().onChange((new InstantCommand(() -> Angle = this.robotAngle.get())));

    }

    @Override
    public double InputLeft() {
        if (LeftStick.button(2).getAsBoolean()) {
            return 0;
        } else {
            return -LeftStick.getY();
        }
    }

    @Override
    public double InputUp() {
        if (LeftStick.button(2).getAsBoolean()) {
            return 0;
        } else {
            return -LeftStick.getX();
        }
    }

    @Override
    public double InputTheta() {
        return -RightStick.getX();
    }

    @Override
    public Translation2d POV() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            if (Utils.isSimulation()) {
                PerspectiveOffset = 90;
            } else {
                DriverStation.getAlliance().ifPresent(allianceColor -> {
                    PerspectiveOffset = (
                            allianceColor == Alliance.Red
                                    ? 180
                                    : 0);
                    m_hasAppliedOperatorPerspective = true;
                });
            }
        }
        Translation2d POV;
        if (LeftStick.button(2).getAsBoolean()) {
            POV = new Translation2d(-LeftStick.getY(), -LeftStick.getX());
        } else {
            POV = AngleToPOV(LeftStick.getHID().getPOV());
        }
        return POV.rotateAround(new Translation2d(0, 0), Rotation2d.fromDegrees(PerspectiveOffset - Angle));
    }

    @Override
    public Trigger Seed() {
        return LeftStick.button(12);
    }

    @Override
    public Trigger Brake() {
        return RightStick.trigger();
    }

    @Override
    public Trigger robotRel() {
        return LeftStick.trigger();
    }
    @Override
    public Trigger autoAlign() {
        return LeftStick.button(3);
    }


}