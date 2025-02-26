package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HeadSystem;

public class AutoShootCommand extends Command {
    private final HeadSystem headSystem;
    private final double speed;
    private final double duration;
    private final Timer timer = new Timer();

    public AutoShootCommand(HeadSystem headSystem, double speed, double duration) {
        this.headSystem = headSystem;
        this.speed = speed;
        this.duration = duration;
        addRequirements(headSystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        headSystem.feedMotor.set(-speed); // ✅ Start shooting
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= duration; // ✅ Stop after duration
    }

    @Override
    public void end(boolean interrupted) {
        headSystem.feedMotor.set(0); // ✅ Ensure motor stops
        timer.stop();
    }
}
