package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLights;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class AlignToLeftPole extends Command {
    private final SwerveSubsystem drivebase;
    private final LimeLights limeLights;

    private final PIDController lateralPID = new PIDController(0.055, 0.0, 0.0); // Left/Right Alignment
    private final PIDController rotationPID = new PIDController(0.015, 0.0, 0.0); // ✅ Lowered Rotation PID
    private final PIDController distancePID = new PIDController(0.15, 0.0, 0.00); // Forward/Backward

    private static final double TARGET_DISTANCE_METERS = 3.00; // 16 inches from tag
    private static final double TARGET_X_METERS = 0.0254; // Shift 1 inch to the right
    private static final double YAW_TOLERANCE = 3; // Degrees
    private static final double LATERAL_TOLERANCE = 0.05; // 5 cm tolerance for lateral alignment
    private static final double POSITION_TOLERANCE = 0.03; // 5 cm tolerance for forward/backward
    private static final double SPEED_LIMIT = 1.0; // Max speed for lateral/forward
    private static final double ROTATION_LIMIT = 1.0; // ✅ Max rotation speed

    public AlignToLeftPole(SwerveSubsystem drivebase, LimeLights limeLights) {
        this.drivebase = drivebase;
        this.limeLights = limeLights;
        addRequirements(limeLights, drivebase);
    }

    @Override
    public void execute() {
        // ✅ Check if an AprilTag is detected
        if (!limeLights.isAprilTagDetected(false)) {
            drivebase.stop();
            return;
        }

        // ✅ Retrieve botPose data **every cycle**
        double[] botPose = limeLights.getAprilTagPose(false);
        if (botPose.length < 18) {
            drivebase.stop();
            return;
        }

        // ✅ Correctly Extract **botPose_TargetSpace** Values
        double lateralOffset = botPose[12];  // Left/Right offset from AprilTag
        double forwardOffset = botPose[13];  // Forward/Backward distance from AprilTag
        double rotationOffset = botPose[5]; // Rotation relative to the tag

        // ✅ Compute Control Outputs
        double lateralSpeed = lateralPID.calculate(lateralOffset, TARGET_X_METERS);
        double forwardSpeed = distancePID.calculate(forwardOffset, TARGET_DISTANCE_METERS);
        double rotationSpeed = rotationPID.calculate(rotationOffset, 0); // Always target 0 yaw

        // ✅ Apply minimum movement thresholds to prevent unnecessary drift
        if (Math.abs(forwardSpeed) < 0.05) forwardSpeed = 0;
        if (Math.abs(lateralSpeed) < 0.05) lateralSpeed = 0;
        if (Math.abs(rotationSpeed) < 0.05) rotationSpeed = 0;

        // ✅ Clamp speeds to avoid excessive movement
        lateralSpeed = Math.max(-SPEED_LIMIT, Math.min(lateralSpeed, SPEED_LIMIT));
        forwardSpeed = Math.max(-SPEED_LIMIT, Math.min(forwardSpeed, SPEED_LIMIT));
        rotationSpeed = Math.max(-ROTATION_LIMIT, Math.min(rotationSpeed, ROTATION_LIMIT));

        // ✅ Debugging: Print All PID Outputs + Rotation Offset
        // System.out.println("Rotation Offset: " + rotationOffset);

        // ✅ Apply All Adjustments Simultaneously
        drivebase.drive(new Translation2d(forwardSpeed, lateralSpeed), rotationSpeed, false);
    }

    @Override
    public boolean isFinished() {
        double[] botPose = limeLights.getAprilTagPose(false);
        if (botPose.length < 18) return false;

        boolean lateralAligned = Math.abs(botPose[12] - TARGET_X_METERS) < LATERAL_TOLERANCE;
        boolean rotationAligned = Math.abs(botPose[5]) < YAW_TOLERANCE;
        boolean forwardAligned = Math.abs(botPose[13] - TARGET_DISTANCE_METERS) < POSITION_TOLERANCE;

        return lateralAligned && rotationAligned && forwardAligned;
    }

    @Override
    public void end(boolean interrupted) {
        // ✅ Stop all movement when command ends
        drivebase.drive(new Translation2d(0, 0), 0, false);
        drivebase.stop();
    }
}
