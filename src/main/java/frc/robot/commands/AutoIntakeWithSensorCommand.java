package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HeadSystem;

public class AutoIntakeWithSensorCommand extends Command {
    private final HeadSystem headSystem;
    private final double speed;

    public AutoIntakeWithSensorCommand(HeadSystem headSystem, double speed) {
        this.headSystem = headSystem;
        this.speed = speed;
        addRequirements(headSystem); // ✅ Ensure no conflicts with other commands
    }

    @Override
    public void initialize() {
        headSystem.feedMotor.set(speed); // ✅ Start intake
    }

    @Override
    public void execute() {
        double sensorValue = headSystem.loadSensor.getDistance().getValueAsDouble() * 1000;
        if (sensorValue < 85) {
            headSystem.feedMotor.set(0); // ✅ Stop intake when object is detected
        }
    }

    @Override
    public boolean isFinished() {
        return headSystem.loadSensor.getDistance().getValueAsDouble() * 1000 < 85; // ✅ Command stops when object is detected
    }

    @Override
    public void end(boolean interrupted) {
        headSystem.feedMotor.set(0); // ✅ Ensure intake stops
    }
}
