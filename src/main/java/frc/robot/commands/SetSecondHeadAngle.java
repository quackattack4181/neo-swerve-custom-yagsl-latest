package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SecondHeadSystem;

public class SetSecondHeadAngle extends Command {
    private final SecondHeadSystem secondHeadSystem;
    private final double targetAngle;
    private final Timer timer = new Timer();

    public SetSecondHeadAngle(double targetAngle, SecondHeadSystem secondHeadSystem) {
        this.targetAngle = targetAngle;
        this.secondHeadSystem = secondHeadSystem;
        addRequirements(secondHeadSystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    } 

    @Override
    public void execute() {
        double currentAngle = secondHeadSystem.secondHeadEncoder.get() * 360;
        double gate1 = targetAngle - 1;
        double gate2 = targetAngle + 1;

        // Move only if we are outside the target range
        if (currentAngle < gate1 || currentAngle > gate2) {
            secondHeadSystem.setSecondHeadAngle(targetAngle);
        }
    }

    @Override
    public boolean isFinished() {
        double currentAngle = secondHeadSystem.secondHeadEncoder.get() * 360;

        // Stop if timeout is reached or head is at target
        return (Math.abs(currentAngle - targetAngle) <= 1) || (timer.get() > 5.0);
    }

    @Override
    public void end(boolean interrupted) {
        secondHeadSystem.runHeadStop();
        timer.stop();
    }
}
