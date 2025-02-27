package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase; // ✅ Import SubsystemBase
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import limelight.networktables.LimelightResults;

public class LimeLightSystem extends SubsystemBase { // ✅ Extend SubsystemBase
    private final SwerveSubsystem swerveSubsystem;
    private boolean sideAlignLeft = true;

    public LimeLightSystem(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
    }

    public void driveForward(double speed) { swerveSubsystem.drive(new Translation2d(speed, 0.00), 0, false); }
    public void driveBackward(double speed) { swerveSubsystem.drive(new Translation2d(-speed, 0.00), 0, false); }
    public void driveLeft(double speed) { swerveSubsystem.drive(new Translation2d(0.0, speed), 0, false); }
    public void driveRight(double speed) { swerveSubsystem.drive(new Translation2d(0.0, -speed), 0, false); }
    public void driveStop() { swerveSubsystem.drive(new Translation2d(0.0, 0.0), 0, false); }

    public void driveTX(double currentTX, double targetTX) {
        if (currentTX > targetTX + 1) { driveRight(0.30); }
        else if (currentTX < targetTX - 1) { driveLeft(0.30); }
        else { driveStop(); }
    }

    public double getCurrentTX() {
        return Robot.getInstance().m_robotContainer.limeLightLeft.getLatestResults()
            .map(result -> result.targets_Fiducials.length > 0 ? result.targets_Fiducials[0].tx : 0)
            .orElse((double) 0);
    }

    public void driveTA(double currentTA, double targetTA) {
        if (currentTA > targetTA + 0.25) { driveBackward(0.20); }
        else if (currentTA < targetTA - 3) { driveForward(0.30); }
        else if (currentTA < targetTA - 5) { driveForward(1.0); }
        else { driveStop(); }
    }

    public double getCurrentTA() {
        return Robot.getInstance().m_robotContainer.limeLightLeft.getLatestResults()
            .map(result -> result.targets_Fiducials.length > 0 ? result.targets_Fiducials[0].ta * 100 : 0)
            .orElse((double) 0);
    }
    

    public void runCenterRobotOnLeftTag() {
        sideAlignLeft = false;
        Robot.getInstance().m_robotContainer.limeLightLeft.getLatestResults().ifPresent(result -> {
            if (result.targets_Fiducials.length > 0) {
                final double currentTX = result.targets_Fiducials[0].tx;
                driveTX(currentTX, -2.00);
            }
        });
    }

    public void runCenterRobotOnRightTag() {
        sideAlignLeft = true;
        Robot.getInstance().m_robotContainer.limeLightRight.getLatestResults().ifPresent(result -> {
            if (result.targets_Fiducials.length > 0) {
                final double currentTX = result.targets_Fiducials[0].tx;
                driveTX(currentTX, -5.00);
            }
        });
    }

    public void runCrawlForward() {
        if (sideAlignLeft) {
            Robot.getInstance().m_robotContainer.limeLightRight.getLatestResults().ifPresent(result -> {
                if (result.targets_Fiducials.length > 0) {
                    final double currentTA = result.targets_Fiducials[0].ta * 100;
                    driveTA(currentTA, 13);
                }
            });
        } else {
            Robot.getInstance().m_robotContainer.limeLightLeft.getLatestResults().ifPresent(result -> {
                if (result.targets_Fiducials.length > 0) {
                    final double currentTA = result.targets_Fiducials[0].ta * 100;
                    driveTA(currentTA, 13);
                }
            });
        }
    }

    @Override
    public void periodic() { // ✅ WPILib calls this every robot loop (~20ms)
        // Limelight could be updated here if needed
    }
}
