package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import limelight.Limelight;


import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import limelight.networktables.LimelightResults;

public class LimeLightSystem {
    private final SwerveSubsystem swerveSubsystem;
    private final XboxController xboxDriver;


    public LimeLightSystem(SwerveSubsystem swerveSubsystem, XboxController xboxDriver) {
        this.swerveSubsystem = swerveSubsystem;
        this.xboxDriver = xboxDriver;
    }

    // TARGETS
    // TX ---> 0.0 OR Greator than -1 and Less than +1      Left: -     Right: +
    // TY ---> 0.0 OR Greator than -1 and Less than +1      Up: +       Down: -
    // TA ---> 0.30

    // swerveSubsystem.drive(new Translation2d(0.2, 0.0), 0, false); // Drive Forward
    // swerveSubsystem.drive(new Translation2d(-0.2, 0.0), 0, false); // Drive Backwards


    public void driveLeft(double speed) {
        swerveSubsystem.drive(new Translation2d(0.0, speed), 0, false);
    };
    public void driveRight(double speed) {
        swerveSubsystem.drive(new Translation2d(0.0, -speed), 0, false);
    };
    public void driveStop() {
        swerveSubsystem.drive(new Translation2d(0.0, 0.0), 0, false);
    };



    public void runCenterRobotOnTag() {
        Robot.getInstance().m_robotContainer.limelight.getLatestResults().ifPresent((LimelightResults result) -> {
            // If a tag is visible run this code.
            if (result.targets_Fiducials.length > 0) {
                final double tagID = result.targets_Fiducials[0].fiducialID; // Tag ID for currently visible Tag

                // Get the tag fiducial values
                final double tx = result.targets_Fiducials[0].tx;
                final double ty = result.targets_Fiducials[0].ty;
                final double ta = result.targets_Fiducials[0].ta;

                // Blue Side Reef April Tags
                if (tagID >= 17 && tagID <= 22 ) {

                    // If not centered drive to the right.
                    if (tx > 1 && tx < 7) { driveRight(0.25); }
                    else if (tx > 7) { driveRight(0.50); }
                    else if (tx > 15) { driveRight(1.50); }

                    // If not centered drive to the left.
                    else if (tx < -1 && tx > -7) { driveLeft(0.25); }
                    else if (tx < -7) { driveLeft(0.50); }
                    else if (tx < -15) { driveLeft(1.50); }

                    // Stop the robot.
                    else { driveStop(); }

                    // prints the current tag and variables to the terminal
                    System.out.println("BLUE TAG ID: "+tagID+" TX: "+String.format("%.3f", tx)+" TY: "+String.format("%.3f", ty)+" TA: "+String.format("%.3f", ta));
                }

                // Red Side Reef April Tags
                if (tagID >= 6 && tagID <= 11 ) {

                    // prints the current tag and variables to the terminal
                    System.out.println("RED TAG ID: "+tagID+" TX: "+String.format("%.3f", tx)+" TY: "+String.format("%.3f", ty)+" TA: "+String.format("%.3f", ta));
                }

            }
        });
    }

    public void update() {

        // If X button is currently held down
        if (xboxDriver.getXButton()) {
            runCenterRobotOnTag();
        }
    }
}


