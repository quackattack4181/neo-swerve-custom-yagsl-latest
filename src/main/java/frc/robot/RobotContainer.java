// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));


  public final AprilTag aprilTag1 = new AprilTag(drivebase, 1); // Replace `1` with your desired tag ID
  public final AprilTag aprilTag2 = new AprilTag(drivebase, 2); // Replace `1` with your desired tag ID
  public final AprilTag aprilTag3 = new AprilTag(drivebase, 3); // Replace `1` with your desired tag ID
  public final AprilTag aprilTag4 = new AprilTag(drivebase, 4); // Replace `1` with your desired tag ID
  public final AprilTag aprilTag5 = new AprilTag(drivebase, 5); // Replace `1` with your desired tag ID
  public final AprilTag aprilTag6 = new AprilTag(drivebase, 6); // Replace `1` with your desired tag ID
  public final AprilTag aprilTag7 = new AprilTag(drivebase, 7); // Replace `1` with your desired tag ID
  public final AprilTag aprilTag8 = new AprilTag(drivebase, 8); // Replace `1` with your desired tag ID
  public final AprilTag aprilTag9 = new AprilTag(drivebase, 9); // Replace `1` with your desired tag ID
  public final AprilTag aprilTag10 = new AprilTag(drivebase, 10); // Replace `1` with your desired tag ID
  public final AprilTag aprilTag11 = new AprilTag(drivebase, 11); // Replace `1` with your desired tag ID
  public final AprilTag aprilTag12 = new AprilTag(drivebase, 12); // Replace `1` with your desired tag ID
  public final AprilTag aprilTag13 = new AprilTag(drivebase, 13); // Replace `1` with your desired tag ID
  public final AprilTag aprilTag14 = new AprilTag(drivebase, 14); // Replace `1` with your desired tag ID
  public final AprilTag aprilTag15 = new AprilTag(drivebase, 15); // Replace `1` with your desired tag ID
  public final AprilTag aprilTag16 = new AprilTag(drivebase, 16); // Replace `1` with your desired tag ID
  public final AprilTag aprilTag17 = new AprilTag(drivebase, 17); // Replace `1` with your desired tag ID
  public final AprilTag aprilTag18 = new AprilTag(drivebase, 18); // Replace `1` with your desired tag ID
  public final AprilTag aprilTag19 = new AprilTag(drivebase, 19); // Replace `1` with your desired tag ID
  public final AprilTag aprilTag20 = new AprilTag(drivebase, 20); // Replace `1` with your desired tag ID
  public final AprilTag aprilTag21 = new AprilTag(drivebase, 21); // Replace `1` with your desired tag ID
  public final AprilTag aprilTag22 = new AprilTag(drivebase, 22); // Replace `1` with your desired tag ID


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                               OperatorConstants.LEFT_X_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                 driverXbox.getHID()::getYButtonPressed,
                                                                 driverXbox.getHID()::getAButtonPressed,
                                                                 driverXbox.getHID()::getXButtonPressed,
                                                                 driverXbox.getHID()::getBButtonPressed);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> -driverXbox.getRightX(),
      () -> -driverXbox.getRightY()); 

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRightX() * -1);

  Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRawAxis(2));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Selected", autoChooser);

    // boolean tagVisible = aprilTag22.isTargetTagVisible();
    // System.out.println("Is tag visible? " + tagVisible);
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    if (DriverStation.isTest())
    {
      driverXbox.b().whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
      drivebase.setDefaultCommand(
          !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
    } else {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.b().whileTrue(
          Commands.deferredProxy(() -> drivebase.driveToPose(
                                     new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)))
                                ));
      driverXbox.y().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)))
                              )); // make follow april tag here.
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());

      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
      drivebase.setDefaultCommand(
          !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);

      // If Target Tag is Visible
        driverXbox.x().onTrue(Commands.runOnce(() -> {
          if (this.aprilTag22.isTargetTagVisible()) {
            System.out.println("Pressed X & Target Visible.");
            // Drive forward with a velocity of 1 meter per second
            drivebase.drive(new Translation2d(1.0, 0.0), 0, false);
        } else {
            System.out.println("Pressed X but Target Not Visible.");
        }
        }));
        
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
