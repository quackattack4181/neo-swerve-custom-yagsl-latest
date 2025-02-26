package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimeLightSystem;

public class AutoCenterOnLeftSideCommand extends Command {
    private final LimeLightSystem limeLightSystem;

    public AutoCenterOnLeftSideCommand(LimeLightSystem limeLightSystem) {
        this.limeLightSystem = limeLightSystem;
        addRequirements(limeLightSystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        limeLightSystem.runCenterRobotOnRightTag(); // ✅ Centers on left-side tags
    }

    @Override
    public void end(boolean interrupted) {
        limeLightSystem.driveStop(); // ✅ Stops when the command ends
    }

    @Override
    public boolean isFinished() {
        return false; // ✅ Runs until PathPlanner cancels it
    }
}
