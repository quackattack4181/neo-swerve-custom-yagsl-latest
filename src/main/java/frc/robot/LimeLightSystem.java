package frc.robot;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase; // ✅ Import SubsystemBase
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class LimeLightSystem extends SubsystemBase { // ✅ Extend SubsystemBase
    private final SwerveSubsystem swerveSubsystem;

    public LimeLightSystem(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
    }

    public void driveStop() { swerveSubsystem.drive(new Translation2d(0.0, 0.0), 0.0, false); }


    public void driveToTarget(double currentTX, double targetTX, double currentTA, double targetTA) {
        double tagID = getCurrentAprilTagIdLeftPole(); // Change to getCurrentAprilTagIdRightPole() if needed
    
        // ✅ If no valid AprilTag detected, stop immediately
        if (tagID <= 0) {
            System.out.println("No valid AprilTag detected. Stopping.");
            driveStop();
            return;
        }
    
        // ✅ Compute strafe speed (TX alignment)
        final double strafeSpeed = (currentTX > targetTX + 1) ? -0.27  // Move right
            : (currentTX < targetTX - 1) ?  0.27   // Move left
            :  0.0;  // Stop moving sideways
    
        // ✅ Compute forward speed (TA alignment)
        final double forwardSpeed = (currentTA > targetTA + 0.25) ? -0.20  // Move backward
            : (currentTA < targetTA - 5) ?  1.0   // Move forward (fast)
            : (currentTA < targetTA - 3) ?  0.50  // Move forward (slow)
            :  0.0;  // Stop moving forward/backward
    
        // ✅ Apply both forward and strafe speeds simultaneously
        swerveSubsystem.drive(new Translation2d(forwardSpeed, strafeSpeed), 0, false);
    }
    

    // public void driveToTarget(double currentTX, double targetTX, double currentTA, double targetTA) {
    //     // ✅ Compute strafe speed (TX alignment)
    //     final double strafeSpeed = (currentTX > targetTX + 1) ? -0.27  // Move right
    //     : (currentTX < targetTX - 1) ?  0.27   // Move left
    //     :  0.0;  // Stop moving sideways

    //     // ✅ Compute forward speed (TA alignment)
    //     final double forwardSpeed = (currentTA > targetTA + 0.25) ? -0.20  // Move backward
    //     : (currentTA < targetTA - 5) ?  1.0   // Move forward (fast)
    //     : (currentTA < targetTA - 3) ?  0.50  // Move forward (slow)
    //     :  0.0;  // Stop moving forward/backward


    //     // ✅ Apply movement & "fake" joystick rotation
    //     // swerveSubsystem.driveCommand(
    //     //     () -> forwardSpeed,   // Forward/backward movement
    //     //     () -> strafeSpeed,    // Left/right strafing
    //     //     () -> headingX,       // Fake right-stick X input (rotation)
    //     //     () -> headingY        // Fake right-stick Y input (rotation)
    //     // ).schedule();;

    
    //     // ✅ Apply both forward and strafe speeds simultaneously
    //     swerveSubsystem.drive(new Translation2d(forwardSpeed, strafeSpeed), 0, false);
    // }
    

    public double getCurrentTXLeftPole() {
        return Robot.getInstance().m_robotContainer.limeLightRight.getLatestResults()
            .map(result -> result.targets_Fiducials.length > 0 ? result.targets_Fiducials[0].tx : 0)
            .orElse((double) 0);
    }


    public double getCurrentTALeftPole() {
        return Robot.getInstance().m_robotContainer.limeLightRight.getLatestResults()
            .map(result -> result.targets_Fiducials.length > 0 ? result.targets_Fiducials[0].ta * 100 : 0)
            .orElse((double) 0);
    }

    // public double getCurrentTXLeftPole() {
    //     var optionalResult = Robot.getInstance().m_robotContainer.limeLightRight.getLatestResults();
    
    //     if (optionalResult.isEmpty()) {
    //         System.out.println("No valid Limelight data found (TX Left).");
    //         return 0.0;
    //     }
    
    //     var result = optionalResult.get(); // ✅ Extract valid data
    
    //     // ✅ Debugging: Print full data to find correct fields
    //     System.out.println("Limelight Data: " + result.toString());
    
    //     // ✅ If result has no targets, return 0
    //     if (result.targets.length == 0) {
    //         System.out.println("No targets found.");
    //         return 0.0;
    //     }
    
    //     return result.targets[0].tx; // ✅ Try accessing TX from the first detected target
    // }
    
    
    
    // public double getCurrentTALeftPole() {
    //     var result = Robot.getInstance().m_robotContainer.limeLightRight.getLatestResults();
    
    //     if (result == null || result.targets_Fiducials.length == 0) {
    //         System.out.println("No targets found (TA Left)");
    //         return 0.0;
    //     }
    
    //     return result.targets_Fiducials[0].ta * 100; // Convert to percentage
    // }


    public double getCurrentTXRightPole() {
        return Robot.getInstance().m_robotContainer.limeLightLeft.getLatestResults()
            .map(result -> result.targets_Fiducials.length > 0 ? result.targets_Fiducials[0].tx : 0)
            .orElse((double) 0);
    }


    public double getCurrentTARightPole() {
        return Robot.getInstance().m_robotContainer.limeLightLeft.getLatestResults()
            .map(result -> result.targets_Fiducials.length > 0 ? result.targets_Fiducials[0].ta * 100 : 0)
            .orElse((double) 0);
    }
    

    public double getCurrentAprilTagIdLeftPole() {
        return Robot.getInstance().m_robotContainer.limeLightRight.getLatestResults()
            .map(result -> result.targets_Fiducials.length > 0 ? result.targets_Fiducials[0].fiducialID : 0) // ✅ Return tag ID, or -1 if none
            .orElse((double) 0);
    }
    public double getCurrentAprilTagIdRightPole() {
        return Robot.getInstance().m_robotContainer.limeLightLeft.getLatestResults()
            .map(result -> result.targets_Fiducials.length > 0 ? result.targets_Fiducials[0].fiducialID : 0) // ✅ Return tag ID, or -1 if none
            .orElse((double) 0);
    }


    

    // NEW HERE
    
    





    


    @Override
    public void periodic() { // ✅ WPILib calls this every robot loop (~20ms)
        // Limelight could be updated here if needed
    }
}
