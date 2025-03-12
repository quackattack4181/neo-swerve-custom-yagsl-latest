package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSystem;

public class MoveClimberToPosition extends Command {
    private final ClimberSystem climberSystem;
    private final double targetPosition;

    public MoveClimberToPosition(ClimberSystem climberSystem, double targetPosition) {
        this.climberSystem = climberSystem;
        this.targetPosition = targetPosition;
    }

    @Override
    public void initialize() {
        System.out.println("Moving Climber to Position: " + targetPosition);
    }

    @Override
    public void execute() {
        climberSystem.setClimbPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        double currentPosition = climberSystem.climbMotorEncoder.getPosition();
        return Math.abs(currentPosition - targetPosition) <= 1;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Climber Movement Completed.");
        climberSystem.runClimbArmStop();
    }
}
