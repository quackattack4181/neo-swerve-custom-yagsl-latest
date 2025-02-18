package frc.robot;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import limelight.Limelight;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import limelight.networktables.LimelightResults;

public class AprilTagActions {
    private final SwerveSubsystem swerveSubsystem;
    private final CommandXboxController xboxDriver;


    public AprilTagActions(SwerveSubsystem swerveSubsystem, CommandXboxController xboxDriver) {
        this.swerveSubsystem = swerveSubsystem;
        this.xboxDriver = xboxDriver;
    }

    public void update() {

        // Get target data
        // Robot.getInstance().m_robotContainer.limelight.getLatestResults().ifPresent((LimelightResults result) -> {
            // If a tag is visible run this code.




            // if (result.targets_Fiducials.length > 0) {

            // // TARGETS
            // // TX ---> 0.0 OR Greator than -1 and Less than +1      Left: -     Right: +
            // // TY ---> 0.0 OR Greator than -1 and Less than +1      Up: +       Down: -
            // // TA ---> 0.30

            // // swerveSubsystem.drive(new Translation2d(0.2, 0.0), 0, false); // Drive Forward
            // // swerveSubsystem.drive(new Translation2d(-0.2, 0.0), 0, false); // Drive Backwards
            // // swerveSubsystem.drive(new Translation2d(0.0, 0.2), 0, false); // Drive Left
            // // swerveSubsystem.drive(new Translation2d(0.0, -0.2), 0, false); // Drive Right



            //     // Tag ID for currently visible Tag
            //     final double tagID = result.targets_Fiducials[0].fiducialID;
            //     final double tx = result.targets_Fiducials[0].tx;
            //     final double ty = result.targets_Fiducials[0].ty;
            //     final double ta = result.targets_Fiducials[0].ta;

            //     // Blue Side Reef April Tags
            //     if (tagID == 17 || tagID == 18 || tagID == 19 || tagID == 20 || tagID == 21 || tagID == 22 ) {

            //         // If X button is currently held down
            //         if (xboxDriver.getXButton()) {



            //             // If not centered drive to the right.
            //             if (tx > 1 && tx < 7) {
            //                 swerveSubsystem.drive(new Translation2d(0.0, -0.25), 0, false); // Drive Right
            //             }
            //             else if (tx > 7) {
            //                 swerveSubsystem.drive(new Translation2d(0.0, -0.50), 0, false); // Drive Right
            //             }
            //             else if (tx > 15) {
            //                 swerveSubsystem.drive(new Translation2d(0.0, -1.50), 0, false); // Drive Right
            //             }





            //             // If not centered drive to the left.
            //             else if (tx < -1 && tx > -7) {
            //                 swerveSubsystem.drive(new Translation2d(0.0, 0.25), 0, false); // Drive Left
            //             }
            //             else if (tx < -7) {
            //                 swerveSubsystem.drive(new Translation2d(0.0, 0.50), 0, false); // Drive Right
            //             }
            //             else if (tx < -15) {
            //                 swerveSubsystem.drive(new Translation2d(0.0, 1.50), 0, false); // Drive Right
            //             }


            //             else {
            //                 // Stop the robot.
            //                 swerveSubsystem.drive(new Translation2d(0.0, 0.0), 0, false);
            //             }
            //         }




                    
            //         // prints the current tag and variables to the terminal
            //         System.out.println("BLUE TAG ID: "+tagID+" TX: "+String.format("%.3f", tx)+" TY: "+String.format("%.3f", ty)+" TA: "+String.format("%.3f", ta));
            //     }

            //     // Red Side Reef April Tags
            //     if (tagID == 6 || tagID == 7 || tagID == 8 || tagID == 9 || tagID == 10 || tagID == 11 ) {

            //         // prints the current tag and variables to the terminal
            //         System.out.println("RED TAG ID: "+tagID+" TX: "+String.format("%.3f", tx)+" TY: "+String.format("%.3f", ty)+" TA: "+String.format("%.3f", ta));
            //     }

            // }
            // // If tag is not visible then run this code.
            // else {
            //     // System.out.println("No April Tag Visible!");
            // }


            
            
        // });
    }
}


