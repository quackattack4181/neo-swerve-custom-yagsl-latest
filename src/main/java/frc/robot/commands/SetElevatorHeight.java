package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSystem;

public class SetElevatorHeight extends Command {
    private final ElevatorSystem elevatorSystem;
    private final double targetHeight;

    public SetElevatorHeight(ElevatorSystem elevatorSystem, double height) {
        this.elevatorSystem = elevatorSystem;
        this.targetHeight = height;
        addRequirements(elevatorSystem);
    }

    @Override
    public void initialize() {
        System.out.println("SetElevatorHeight Started: Moving to " + targetHeight);
    }

    @Override
    public void execute() {
        elevatorSystem.setElevatorPosition(targetHeight);  // ✅ Non-blocking function call
    }

    @Override
    public boolean isFinished() {
        double currentHeight = elevatorSystem.ElevatorEncoderLeft.getPosition() * 360;
        return Math.abs(currentHeight - targetHeight) <= 10;
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSystem.runElevatorStop();
        System.out.println("SetElevatorHeight Finished at height: " + elevatorSystem.ElevatorEncoderLeft.getPosition() * 360);
    }
}