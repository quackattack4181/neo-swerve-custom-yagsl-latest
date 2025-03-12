package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLights extends SubsystemBase {
    private static final String LEFT_LIMELIGHT = "limelight-left";
    private static final String RIGHT_LIMELIGHT = "limelight-right";

    public LimeLights() {}

    /** Gets the AprilTag botPose from the selected Limelight */
    public double[] getAprilTagPose(boolean useLeftLimelight) {
        String limelightName = useLeftLimelight ? LEFT_LIMELIGHT : RIGHT_LIMELIGHT;
        NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);

        // Get ALL botpose values
        double[] botPose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

        // Print the entire array for debugging
        // System.out.println(limelightName + " botPose: " + Arrays.toString(botPose));

        return botPose;
    }


    public void disableCameraStream() {
        NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("stream").setNumber(0);
        NetworkTableInstance.getDefault().getTable("limelight-right").getEntry("stream").setNumber(0);
    }

    public void enableVisionProcessing() {
        NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("camMode").setNumber(0);
        NetworkTableInstance.getDefault().getTable("limelight-right").getEntry("camMode").setNumber(0);
    }

    /** Checks if an AprilTag is detected */
    public boolean isAprilTagDetected(boolean useLeftLimelight) {
        String limelightName = useLeftLimelight ? LEFT_LIMELIGHT : RIGHT_LIMELIGHT;
        NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);
    
        
        // Get target valid (tv): 1.0 = detected, 0.0 = no tag
        return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
    }
    
    
}
