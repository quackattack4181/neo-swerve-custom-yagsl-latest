// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoCenterOnLeftSideCommand;
import frc.robot.commands.AutoCenterOnRightSideCommand;
import frc.robot.commands.AutoIntakeWithSensorCommand;
import frc.robot.commands.AutoSetPositionCommand;
import frc.robot.commands.AutoSetPositionCommandLoad;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.MoveClimberToPosition;
import frc.robot.commands.SetElevatorHeight;
import frc.robot.commands.SetHeadAngle;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;

import limelight.Limelight;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
// import com.ctre.phoenix6.hardware.CANrange;
// import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
  "swerve/neo"));

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverOne = new CommandXboxController(0);
  private final CommandXboxController driverTwo = new CommandXboxController(1);

  public HeadSystem HeadSystem = new HeadSystem();
  public ElevatorSystem ElevatorSystem = new ElevatorSystem();
  public ClimberSystem ClimberSystem = new ClimberSystem();                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
  public LimeLightSystem LimeLightSystem  = new LimeLightSystem(drivebase);

  public Limelight limeLightLeft = new Limelight("limelight-left");
  public Limelight limeLightRight = new Limelight("limelight-right");


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(-driverOne.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), // <<<===== CHANGED from -
      () -> MathUtil.applyDeadband(-driverOne.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), // <<<===== CHANGED from -
      () -> -driverOne.getRightX(), // <<<===== CHANGED from -
      () -> -driverOne.getRightY()); // <<<===== CHANGED from -

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(driverOne.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverOne.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
      () -> driverOne.getRightX() * -1);

  Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
      () -> MathUtil.applyDeadband(driverOne.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverOne.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> driverOne.getRawAxis(2));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    CameraServer.startAutomaticCapture();

    // ✅ Register Commands for PathPlanner
    NamedCommands.registerCommand("SetL2", new AutoSetPositionCommand(HeadSystem, ElevatorSystem, HeadSystem.headAngleL2, ElevatorSystem.elevatorPositionL2));
    NamedCommands.registerCommand("SetL3", new AutoSetPositionCommand(HeadSystem, ElevatorSystem, HeadSystem.headAngleL3, ElevatorSystem.elevatorPositionL3));
    NamedCommands.registerCommand("SetL4", new AutoSetPositionCommand(HeadSystem, ElevatorSystem, HeadSystem.headAngleL4, ElevatorSystem.elevatorPositionL4));
    NamedCommands.registerCommand("SetLoad", new AutoSetPositionCommandLoad(HeadSystem, ElevatorSystem, HeadSystem.baseAngle, ElevatorSystem.ElevatorHeightMin));

    // ✅ Register "AutoIntake" command for PathPlanner
    NamedCommands.registerCommand("AutoIntake", new AutoIntakeWithSensorCommand(HeadSystem, HeadSystem.feedMotorSpeed));

    // ✅ Register "AutoShoot" command for PathPlanner
    NamedCommands.registerCommand("AutoShoot", new AutoShootCommand(HeadSystem, -HeadSystem.feedMotorSpeed, 1.0));

    // ✅ Register Commands for PathPlanner (Updated)
    NamedCommands.registerCommand("CenterOnLeftSide", new AutoCenterOnLeftSideCommand(LimeLightSystem));  // ✅ Aligns TX & TA
    NamedCommands.registerCommand("CenterOnRightSide", new AutoCenterOnRightSideCommand(LimeLightSystem)); // ✅ Aligns TX & TA


    // Configure the trigger bindings
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Selected", autoChooser);

    boolean isBlueAlliance = DriverStation.getAlliance()
            .map(alliance -> alliance == DriverStation.Alliance.Blue)
            .orElse(false); // Default to Red if unknown

    // ✅ Add mirrored PathPlanner autos to the chooser
    autoChooser.setDefaultOption("Middle", new PathPlannerAuto("Middle", isBlueAlliance));
    autoChooser.addOption("Left", new PathPlannerAuto("Left", isBlueAlliance));
    autoChooser.addOption("Right", new PathPlannerAuto("Right", isBlueAlliance));
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */

  private void configureBindings() {
    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);




    // Zero the gyro when driverOne presses A
    driverOne.a().onTrue(drivebase.runOnce(drivebase::zeroGyro));

    // ✅ Intake forward while Right Trigger is held, stop when released
    driverTwo.rightTrigger(0.5)
    .whileTrue(new RunCommand(() -> HeadSystem.feedMotor.set(HeadSystem.feedMotorSpeed), HeadSystem))
    .onFalse(new InstantCommand(() -> HeadSystem.feedMotor.set(HeadSystem.off), HeadSystem));

    // ✅ Intake reverse while Left Trigger is held, stop when released
    driverTwo.leftTrigger(0.5)
    .whileTrue(new RunCommand(() -> HeadSystem.feedMotor.set(-HeadSystem.feedMotorSpeed), HeadSystem))
    .onFalse(new InstantCommand(() -> HeadSystem.feedMotor.set(HeadSystem.off), HeadSystem));




    // ✅ Intake runs while RB is held and object is detected (`sensorValue < 85`).
    driverTwo.rightBumper().whileTrue(
      new RunCommand(() -> {
          double sensorValue = HeadSystem.loadSensor.getDistance().getValueAsDouble() * 1000;

          if (sensorValue > 85) { // ✅ Object detected → Keep running intake
              HeadSystem.feedMotor.set(HeadSystem.feedMotorSpeed);
          } else { 
              HeadSystem.feedMotor.set(HeadSystem.off); // ✅ Object loaded → Stop intake
              
              // ✅ Schedule the sequential command when intake stops
              new SequentialCommandGroup(
                  new SetHeadAngle(HeadSystem.headAngleL2, HeadSystem),
                  new SetElevatorHeight(ElevatorSystem, ElevatorSystem.elevatorPositionL2),
                  new SetHeadAngle(HeadSystem.headAngleL2, HeadSystem)
              ).schedule();
          }
      }, HeadSystem)
    ).onFalse(
      new InstantCommand(() -> HeadSystem.feedMotor.set(HeadSystem.off), HeadSystem) // ✅ Stop intake when RB is released
    );





    // ✅ Head joystick control (Right Stick Y moves head up/down)
    // ✅ Move head to max out angle when pushing joystick forward (Right Stick Y < -0.5)

    new Trigger(() -> driverTwo.getRightY() < -0.5)
    .whileTrue(new RunCommand(() -> HeadSystem.setHeadAngle(HeadSystem.headMaxOutAngle), HeadSystem));

    // ✅ Move head to base angle when pulling joystick back (Right Stick Y > 0.5)
    new Trigger(() -> driverTwo.getRightY() > 0.5)
    .whileTrue(new RunCommand(() -> HeadSystem.setHeadAngle(HeadSystem.baseAngle), HeadSystem));

    // ✅ Stop head movement when joystick is centered (between -0.5 and 0.5)
    new Trigger(() -> Math.abs(driverTwo.getRightY()) <= 0.5)
    .onTrue(new InstantCommand(HeadSystem::runHeadStop, HeadSystem));



    // ============== DRIVER TWO BELOW

    // Position Commands
    driverTwo.a().onTrue(
    new SequentialCommandGroup(
        new SetHeadAngle(HeadSystem.headAngleL2, HeadSystem),
        new SetElevatorHeight(ElevatorSystem, ElevatorSystem.elevatorPositionL2),
        new SetHeadAngle(HeadSystem.headAngleL2, HeadSystem)
    ));
    driverTwo.b().onTrue(
    new SequentialCommandGroup(
        new SetHeadAngle(HeadSystem.headAngleL3, HeadSystem),
        new SetElevatorHeight(ElevatorSystem, ElevatorSystem.elevatorPositionL3),
        new SetHeadAngle(HeadSystem.headAngleL3, HeadSystem)
    ));
    driverTwo.y().onTrue(
    new SequentialCommandGroup(
        new SetHeadAngle(HeadSystem.headAngleL4, HeadSystem),
        new SetElevatorHeight(ElevatorSystem, ElevatorSystem.elevatorPositionL4),
        new SetHeadAngle(HeadSystem.headAngleL4, HeadSystem)
    ));
    driverTwo.x().onTrue(
    new SequentialCommandGroup(
        new SetElevatorHeight(ElevatorSystem, ElevatorSystem.ElevatorHeightMin),
        new SetHeadAngle(HeadSystem.baseAngle, HeadSystem)
    ));




    // Left Bumper controls elevator movement with joystick
    driverTwo.leftBumper().whileTrue(
        new RunCommand(() -> {
            double leftY = driverTwo.getLeftY();
            if (leftY < -0.50) {
                ElevatorSystem.runElevatorUp(ElevatorSystem.elevatorSpeed);
            } else if (leftY > 0.50) {
                ElevatorSystem.runElevatorDown(ElevatorSystem.elevatorSpeed);
            } else if (driverTwo.leftStick().getAsBoolean()) { // Left Stick Button Override
                ElevatorSystem.ElevatorLeft.set(-ElevatorSystem.elevatorSpeedSlow);
                ElevatorSystem.ElevatorRight.set(ElevatorSystem.elevatorSpeedSlow);
            } else {
                ElevatorSystem.runElevatorStop();
            }
        }, ElevatorSystem)
    );

    driverTwo.back().onTrue(
    new InstantCommand(() -> {
      ElevatorSystem.runElevatorReset();
    }, ElevatorSystem) // ✅ Now works because ElevatorSystem is a SubsystemBase
  );



  // D-Pad for driverTwo
  // ✅ D-Pad RIGHT (90°) -> Center on Left Tag
  driverTwo.povRight().whileTrue(
      new RunCommand(() -> ClimberSystem.runClimbArmOutward())
  ).onFalse(
    new InstantCommand(() -> ClimberSystem.runClimbArmStop())
  );

  // ✅ D-Pad LEFT (270°) -> Center on Right Tag
  driverTwo.povLeft().whileTrue(
      new RunCommand(() -> ClimberSystem.runClimbArmInward())
  ).onFalse(
    new InstantCommand(() -> ClimberSystem.runClimbArmStop())
  );
  
  driverTwo.povUp().onTrue(new MoveClimberToPosition(ClimberSystem, -350));
  driverTwo.povDown().onTrue(new MoveClimberToPosition(ClimberSystem, 0.0));

  // ✅ START Button -> Reset Climber Encoder to 0
  driverTwo.start().onTrue(
    new InstantCommand(() -> ClimberSystem.climbMotorEncoder.setPosition(0.0), ClimberSystem)
  );



  // Limelight controls.

  // ✅ D-Pad RIGHT (90°) -> Auto Align to Left-Side AprilTags (Right-Side of Field)
  driverOne.povRight()
  .whileTrue(new AutoCenterOnRightSideCommand(LimeLightSystem))
  .onFalse(new InstantCommand(() -> LimeLightSystem.driveStop(), LimeLightSystem));

  // ✅ D-Pad LEFT (270°) -> Auto Align to Right-Side AprilTags (Left-Side of Field)
  driverOne.povLeft()
  .whileTrue(new AutoCenterOnLeftSideCommand(LimeLightSystem))
  .onFalse(new InstantCommand(() -> LimeLightSystem.driveStop(), LimeLightSystem));







}


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

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




//ID 36 for climber