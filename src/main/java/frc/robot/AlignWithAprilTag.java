package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class AlignWithAprilTag extends Command {
    private final SwerveSubsystem swerveSubsystem;

    // PID controllers for x, y, and rotation alignment
    private final PIDController xController = new PIDController(0.5, 0, 0); // Tune these values
    private final PIDController yController = new PIDController(0.5, 0, 0); // Tune these values
    private final PIDController rotationController = new PIDController(0.5, 0, 0); // Tune these values

    // Desired distance and alignment parameters
    private final double targetDistance = 1.0; // Desired distance in meters from the tag
    private final double targetYaw = 0.0; // Desired yaw (0 = face directly toward the tag)

    public AlignWithAprilTag(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);

        // Configure PID controllers
        xController.setTolerance(0.05); // 5 cm tolerance
        yController.setTolerance(0.05); // 5 cm tolerance
        rotationController.setTolerance(2.0); // 2 degrees tolerance
    }

    @Override
    public void execute() {
        // Retrieve pose data from Limelight
        var table = NetworkTableInstance.getDefault().getTable("limelight");
        double[] botPose = table.getEntry("botpose").getDoubleArray(new double[6]);
        double tagID = table.getEntry("tid").getDouble(-1); // Get visible tag ID

        // Check if AprilTag 22 is visible
        if (botPose.length < 6 || tagID != 22) {
            swerveSubsystem.stop();
            return;
        }

        // Extract pose data
        double currentX = botPose[0]; // Robot's X position relative to the tag (meters)
        double currentY = botPose[1]; // Robot's Y position relative to the tag (meters)
        double currentYaw = botPose[5]; // Robot's yaw relative to the tag (degrees)

        // Calculate errors
        double xError = currentX - targetDistance; // Distance error (meters)
        double yError = currentY; // Sideways alignment error (meters)
        double yawError = Math.toRadians(currentYaw - targetYaw); // Rotation alignment error (convert to radians)

        // Compute control outputs using PID controllers
        double forwardSpeed = -xController.calculate(xError);
        double strafeSpeed = -yController.calculate(yError);
        double rotationSpeed = -rotationController.calculate(yawError);

        // Create Translation2d for drive input
        Translation2d translation = new Translation2d(forwardSpeed, strafeSpeed);

        // Feed velocities to swerve drive
        swerveSubsystem.drive(translation, rotationSpeed, true);
    }

    @Override
    public boolean isFinished() {
        // Finish when all errors are within tolerance
        return xController.atSetpoint() && yController.atSetpoint() && rotationController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        swerveSubsystem.stop();
    }
}
