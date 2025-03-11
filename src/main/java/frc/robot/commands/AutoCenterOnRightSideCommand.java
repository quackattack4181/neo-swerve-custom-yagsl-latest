package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimeLightSystem;

public class AutoCenterOnRightSideCommand extends Command {
    private final LimeLightSystem limeLightSystem;
    private final double targetTX = -6.0; // 2.0; // ✅ Adjust TX target for right-side AprilTags
    private final double targetTA = 12.5; // ✅ Adjust TA target for correct distance

    public AutoCenterOnRightSideCommand(LimeLightSystem limeLightSystem) {
        this.limeLightSystem = limeLightSystem;
        addRequirements(limeLightSystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double currentTX = limeLightSystem.getCurrentTXRightPole();
        double currentTA = limeLightSystem.getCurrentTARightPole();
        double tagID = limeLightSystem.getCurrentAprilTagIdRightPole();

        // ✅ Debugging: Print values
        System.out.println("R - TX: " + currentTX + ", TA: " + currentTA + ", TagID: " + tagID);

        // ✅ Only move if a valid AprilTag is detected
        if (tagID > 0) {
            limeLightSystem.driveToTarget(currentTX, targetTX, currentTA, targetTA);
        } else {
            limeLightSystem.driveStop(); // Stop if no tag found
        }
    }

    @Override
    public boolean isFinished() {
        double currentTX = limeLightSystem.getCurrentTXRightPole();
        double currentTA = limeLightSystem.getCurrentTARightPole();

        // ✅ Stops when both TX and TA are within target range
        return Math.abs(currentTX - targetTX) <= 1 && 
               !(currentTA > targetTA + 0.25 || currentTA < targetTA - 3 || currentTA < targetTA - 5);
    }

    @Override
    public void end(boolean interrupted) {
        limeLightSystem.driveStop(); // ✅ Stop movement when command ends
    }
}
