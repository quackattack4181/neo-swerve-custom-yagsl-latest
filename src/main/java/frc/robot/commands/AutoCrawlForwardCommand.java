package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimeLightSystem;

public class AutoCrawlForwardCommand extends Command {
    private final LimeLightSystem limeLightSystem;
    private final double targetTA = 13.0; // ✅ Adjust as needed for correct distance

    public AutoCrawlForwardCommand(LimeLightSystem limeLightSystem) {
        this.limeLightSystem = limeLightSystem;
        addRequirements(limeLightSystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double currentTA = limeLightSystem.getCurrentTA(); // ✅ Get TA (target area) from Limelight
        limeLightSystem.driveTA(currentTA, targetTA);
    }

    @Override
    public boolean isFinished() {
        double currentTA = limeLightSystem.getCurrentTA();
        return !(currentTA > targetTA + 0.25 || currentTA < targetTA - 3 || currentTA < targetTA - 5);
        // ✅ Ends when it reaches the stopping condition in driveTA()
    }

    @Override
    public void end(boolean interrupted) {
        limeLightSystem.driveStop(); // ✅ Stop movement when command ends
    }
}
