package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimeLightSystem;

public class AutoCenterOnLeftSideCommand extends Command {
    private final LimeLightSystem limeLightSystem;
    private final double targetTX = -7.0;  // ✅ Adjust TX target for left-side AprilTags
    private final double targetTA = 12.5; // ✅ Adjust TA target for correct distance

    public AutoCenterOnLeftSideCommand(LimeLightSystem limeLightSystem) {
        this.limeLightSystem = limeLightSystem;
        addRequirements(limeLightSystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double currentTX = limeLightSystem.getCurrentTXLeftPole();
        double currentTA = limeLightSystem.getCurrentTALeftPole();
        double tagID = limeLightSystem.getCurrentAprilTagIdLeftPole();

        limeLightSystem.driveToTarget(currentTX, targetTX, currentTA, targetTA);
    }

    @Override
    public boolean isFinished() {
        double currentTX = limeLightSystem.getCurrentTXLeftPole();
        double currentTA = limeLightSystem.getCurrentTALeftPole();

        // ✅ Stops when both TX and TA are within target range
        return Math.abs(currentTX - targetTX) <= 1 && 
               !(currentTA > targetTA + 0.25 || currentTA < targetTA - 3 || currentTA < targetTA - 5);
    }

    @Override
    public void end(boolean interrupted) {
        limeLightSystem.driveStop(); // ✅ Stop movement when command ends
    }
}
