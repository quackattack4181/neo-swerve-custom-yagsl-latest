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

        double headingX = 0.00;
        double headingY = 0.00;

        // Direction	                    Right Stick X	         Right Stick Y
        // Face Forward	            90°	    0.0	                     1.0
        // Face Forward-Right	    30°	    0.866	                 0.5
        // Face Backward-Right	    330°	0.866	                 -0.5
        // Face Backward	        270°	0.0	                     -1.0
        // Face Backward-Left	    210°	-0.866	                 -0.5
        // Face Forward-Left	    150°	-0.866	                 0.5

        
        // if (tagID == 18 || tagID == 7)       { headingX = 0.00;   headingY = 1.00;}
        // else if (tagID == 19 || tagID == 6)  { headingX = 0.866;  headingY = 0.50;}
        // else if (tagID == 20 || tagID == 11) { headingX = 0.866;  headingY = -0.50;}
        // else if (tagID == 21 || tagID == 10) { headingX = 0.00;   headingY = -1.00;}
        // else if (tagID == 22 || tagID == 9)  { headingX = -0.866; headingY = -0.50;}
        // else if (tagID == 17 || tagID == 8)  { headingX = -0.866; headingY = 0.50;}

        // ✅ NEW: Aligns TX & TA simultaneously
        limeLightSystem.driveToTarget(currentTX, targetTX, currentTA, targetTA);
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
