package frc.robot.subsystems.Auton;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ReefChooser {
    public static String Choose(String AB, Pose2d CurrentPose, Alliance currentAlliance) {
        
            int Side = 6;
            if (currentAlliance == DriverStation.Alliance.Blue) {
                // blue
                if (CurrentPose.getX() > 4.478) {
                    // right side of blue
                    if (CurrentPose.getY() > (0.6 * (CurrentPose.getX() - 4.478)) + 3.987) {
                        // top right of blue
                        Side = 10;
                    } else if (CurrentPose.getY() < (-0.6 * (CurrentPose.getX() - 4.478)) + 3.987) {
                        // bottom Right of blue
                        Side = 2;
                    } else {
                        Side = 12;
                    }
                } else {
                    // left side of blue
                    if (CurrentPose.getY() > (-0.6 * (CurrentPose.getX() - 4.478)) + 3.987) {
                        // top left of blue
                        Side = 8;
                    } else if (CurrentPose.getY() < (0.6 * (CurrentPose.getX() - 4.478)) + 3.987) {
                        // bottom left of blue
                        Side = 4;
                    } else {
                        Side = 6;
                    }
                }
            } else {
                // red
                if (CurrentPose.getX() < 13.102) {
                    // left side of red
                    if (CurrentPose.getY() > (-0.6 * (CurrentPose.getX() - 13.102)) + 3.987) {
                        // top right of blue
                        Side = 2;
                    } else if (CurrentPose.getY() < (0.6 * (CurrentPose.getX() - 13.102)) + 3.987) {
                        // bottom Right of blue
                        Side = 10;
                    } else {
                        Side = 12;
                    }
                } else {
                    // left side of blue
                    if (CurrentPose.getY() > (0.6 * (CurrentPose.getX() - 13.102)) + 3.987) {
                        // top left of blue
                        Side = 4;
                    } else if (CurrentPose.getY() < (-0.6 * (CurrentPose.getX() - 13.102)) + 3.987) {
                        // bottom left of blue
                        Side = 8;
                    } else {
                        Side = 6;
                    }
                }

            }

            return "R" + Side + AB;
        }
    }
