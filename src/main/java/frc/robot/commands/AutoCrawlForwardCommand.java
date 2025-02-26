package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimeLightSystem;

public class AutoCrawlForwardCommand extends Command {
    private final LimeLightSystem limeLightSystem;

    public AutoCrawlForwardCommand(LimeLightSystem limeLightSystem) {
        this.limeLightSystem = limeLightSystem;
        addRequirements(limeLightSystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        limeLightSystem.runCrawlForward();
    }

    @Override
    public void end(boolean interrupted) {
        limeLightSystem.driveStop(); // ✅ Stops movement when command ends
    }

    @Override
    public boolean isFinished() {
        return false; // ✅ Runs until PathPlanner cancels it
    }
}
