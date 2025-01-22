package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AprilTag {
    private final SwerveSubsystem swerveSubsystem;

    private final int targetTagID; // The specific AprilTag ID to track
    private boolean isTagVisible; // Boolean to track visibility of the target tag

    public AprilTag(SwerveSubsystem swerveSubsystem, int targetTagID) {
        this.swerveSubsystem = swerveSubsystem;
        this.targetTagID = targetTagID;
        this.isTagVisible = false;
    }

    public void update() {
        var table = NetworkTableInstance.getDefault().getTable("limelight");
        double[] botPose = table.getEntry("botpose").getDoubleArray(new double[6]);
        double tagID = table.getEntry("tid").getDouble(-1); // Get the currently visible tag ID

        // Check if the specific AprilTag is visible
        if (botPose.length >= 6 && tagID == targetTagID) {
            isTagVisible = true;
        } else {
            isTagVisible = false;
        }

        // Optional: You can add driving logic here if needed
    }

    public boolean isTargetTagVisible() {
        return isTagVisible; // Expose the boolean value to other classes
    }
}



//
//