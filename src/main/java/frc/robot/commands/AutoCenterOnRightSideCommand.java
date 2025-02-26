package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimeLightSystem;

public class AutoCenterOnRightSideCommand extends Command {
    private final LimeLightSystem limeLightSystem;
    private final double targetTX = -6.0; // ✅ Adjust target TX as needed

    public AutoCenterOnRightSideCommand(LimeLightSystem limeLightSystem) {
        this.limeLightSystem = limeLightSystem;
        addRequirements(limeLightSystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double currentTX = limeLightSystem.getCurrentTX(); // ✅ Get TX from Limelight
        limeLightSystem.driveTX(currentTX, targetTX);
    }

    @Override
    public boolean isFinished() {
        double currentTX = limeLightSystem.getCurrentTX();
        return Math.abs(currentTX - targetTX) <= 1; // ✅ Stops when within ±1 of target
    }

    @Override
    public void end(boolean interrupted) {
        limeLightSystem.driveStop(); // ✅ Stop movement when command ends
    }
}
