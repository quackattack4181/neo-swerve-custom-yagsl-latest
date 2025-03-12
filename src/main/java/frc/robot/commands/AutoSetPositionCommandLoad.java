package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.HeadSystem;

public class AutoSetPositionCommandLoad extends SequentialCommandGroup {
    public AutoSetPositionCommandLoad(HeadSystem headSystem, ElevatorSystem elevatorSystem, double headAngle, double elevatorHeight) {
        addCommands(
            new SetElevatorHeight(elevatorSystem, elevatorHeight),
            new SetHeadAngle(headAngle, headSystem) // Moves head again after elevator movement
        );
    }
}

