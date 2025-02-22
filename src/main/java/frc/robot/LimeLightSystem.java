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

    public void driveForward(double speed) {
        swerveSubsystem.drive(new Translation2d(speed, 0.00), 0, false);
    };
    public void driveBackward(double speed) {
        swerveSubsystem.drive(new Translation2d(-speed, 0.00), 0, false);
    };


    public void driveLeft(double speed) {
        swerveSubsystem.drive(new Translation2d(0.0, speed), 0, false);
    };
    public void driveRight(double speed) {
        swerveSubsystem.drive(new Translation2d(0.0, -speed), 0, false);
    };

    public void driveStop() {
        swerveSubsystem.drive(new Translation2d(0.0, 0.0), 0, false);
    };

    public void driveTX(double currentTX, double targetTX) {
        // If not centered drive to the right.
        if (currentTX > targetTX + 1 && currentTX < targetTX + 5) { driveRight(0.20); }
        else if (currentTX > targetTX + 5) { driveRight(0.40); }
        else if (currentTX > targetTX + 10) { driveRight(0.80); }
        else if (currentTX > targetTX + 15) { driveRight(1.60); }

        // If not centered drive to the left.
        else if (currentTX < targetTX - 1 && currentTX > targetTX - 5) { driveLeft(0.20); }
        else if (currentTX < targetTX - 5) { driveLeft(0.40); }
        else if (currentTX < targetTX - 10) { driveLeft(0.80); }
        else if (currentTX < targetTX - 15) { driveLeft(1.60); }

        // Stop the robot.
        else { driveStop(); }
    }


    public void driveTA(double currentTA, double targetTA) {
        // If not centered drive to the right.
        if (currentTA > targetTA + 0.25) { driveBackward(0.20); }
        else if (currentTA < targetTA - 7) { driveForward(0.75); }
        else if (currentTA < targetTA - 5) { driveForward(0.50); }
        else if (currentTA < targetTA - 3) { driveForward(0.25); }

        // Stop the robot.
        else { driveStop(); }
    }


    public void runCenterRobotOnLeftTag() {
        Robot.getInstance().m_robotContainer.limelight.getLatestResults().ifPresent((LimelightResults result) -> {
            // If a tag is visible run this code.
            if (result.targets_Fiducials.length > 0) {
                final double tagID = result.targets_Fiducials[0].fiducialID; // Tag ID for currently visible Tag

                // Get the tag fiducial values
                final double currentTX = result.targets_Fiducials[0].tx;
                // final double currentTY = result.targets_Fiducials[0].ty;
                // final double currentTA = result.targets_Fiducials[0].ta;

                // Blue Side Reef April Tags
                if (tagID >= 17 && tagID <= 22 ) {
                    driveTX(currentTX, -5.00);
                }

                // Red Side Reef April Tags
                if (tagID >= 6 && tagID <= 11 ) {
                    driveTX(currentTX, -5.00);
                }

            }
        });
    }
    // public void runCenterRobotOnRightTag() {
    //     Robot.getInstance().m_robotContainer.limelight.getLatestResults().ifPresent((LimelightResults result) -> {
    //         // If a tag is visible run this code.
    //         if (result.targets_Fiducials.length > 0) {
    //             final double tagID = result.targets_Fiducials[0].fiducialID; // Tag ID for currently visible Tag

    //             // Get the tag fiducial values
    //             final double currentTX = result.targets_Fiducials[0].tx;
    //             // final double currentTY = result.targets_Fiducials[0].ty;
    //             // final double currentTA = result.targets_Fiducials[0].ta;

    //             // Blue Side Reef April Tags
    //             if (tagID >= 17 && tagID <= 22 ) {
    //                 driveTX(currentTX, -20.00);
    //             }

    //             // Red Side Reef April Tags
    //             if (tagID >= 6 && tagID <= 11 ) {
    //                 driveTX(currentTX, -20.00);
    //             }

    //         }
    //     });
    // }

    public void runCrawlForward() {
        Robot.getInstance().m_robotContainer.limelight.getLatestResults().ifPresent((LimelightResults result) -> {
            // If a tag is visible run this code.
            if (result.targets_Fiducials.length > 0) {
                final double tagID = result.targets_Fiducials[0].fiducialID; // Tag ID for currently visible Tag

                // Get the tag fiducial values
                // final double currentTX = result.targets_Fiducials[0].tx;
                // final double currentTY = result.targets_Fiducials[0].ty;
                final double currentTA = result.targets_Fiducials[0].ta*100;

                // Blue Side Reef April Tags
                if (tagID >= 17 && tagID <= 22 ) {

                    driveTA(currentTA, 13);}

                // Red Side Reef April Tags
                if (tagID >= 6 && tagID <= 11 ) {

                    driveTA(currentTA, 13);
                }

            }
        });
    }

    public void update() {

        // Right side of POV
        if (xboxDriver.getPOV() == 0 ) {
            System.out.println("Forward");
            runCrawlForward();
        }
        // Right side of POV
        if (xboxDriver.getPOV() == 90 ) {
            System.out.println("Right");
            // runCenterRobotOnRightTag();

        }
        // Left side of POV
        if (xboxDriver.getPOV() == 270 ) {
            System.out.println("Left");
            runCenterRobotOnLeftTag();
        }

    }
}


