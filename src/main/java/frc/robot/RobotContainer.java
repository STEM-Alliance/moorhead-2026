// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveState;
import frc.robot.subsystems.ShooterSubsystem.ShootMode;
import frc.robot.subsystems.ShooterSubsystem.ShootMotor;
import frc.robot.util.DriveStateHandler;
import frc.robot.util.Telemetry;
import frc.robot.util.TunerConstants;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final CommandXboxController driverXbox = new CommandXboxController(
  // ControllerConstants.DRIVER_CONTROLLER_PORT);
  // private final CommandXboxController operatorXbox = new CommandXboxController(
  // ControllerConstants.OPERATOR_CONTROLLER_PORT);
  // private final SwerveSubsystem swerveDriveSubsystem = new SwerveSubsystem();
  // private final DriveCommand driveCommand = new
  // DriveCommand(swerveDriveSubsystem, driverXbox.getHID());

  private final CommandXboxController driveController = new CommandXboxController(
      ControllerConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController opController = new CommandXboxController(
      ControllerConstants.OPERATOR_CONTROLLER_PORT);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final DriveStateHandler driveStateHandler = new DriveStateHandler(drivetrain, driveController.getHID());
  private final Telemetry logger = new Telemetry(DriveConstants.MAX_ROBOT_VELOCITY);
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    shooterSubsystem
        .setShooterMotors(
            new ShootMotor(15, false), // Primary
            new ShootMotor(16, true) // Follower
        )
        .setHoodMotor(
          new ShootMotor(17, false)
        )
        .setShootVoltage(1)
        .finish();

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> driveStateHandler.getSwerveRequest()));
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    driveController.button(8).onTrue(new InstantCommand(() -> {
      System.out.println("Reset Rotation");
      drivetrain.resetRotation(DriveConstants.GYRO_ANGLE_OFFSET);
    }));

    opController.rightBumper().onTrue(new InstantCommand(() -> {
      shooterSubsystem.requestShootMode(ShootMode.Shooting);
      shooterSubsystem.speedGreaterThen(1000, (rpm) -> {
        // Release Ammo Door? IDK
        System.out.println("Shooter Ready To Shoot! (" + rpm + " > 1000)");
        shooterSubsystem.getShooterMotors().primary().getDeviceId();
        shooterSubsystem.getShooterMotors().follower().getDeviceId();
      });
    })).onFalse(new InstantCommand(() -> shooterSubsystem.requestShootMode(ShootMode.Idle)));

    driveController.x().onTrue(new InstantCommand(() -> drivetrain.setDriveState(DriveState.Locked))).onFalse(
        new InstantCommand(() -> drivetrain.setDriveState(DriveState.Free)));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true));

    drivetrain.registerTelemetry(logger::telemeterize);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    final var idle = new SwerveRequest.Idle();
    return Commands.sequence(
        // Reset our field centric heading to match the robot
        // facing away from our alliance station wall (0 deg).
        drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        // Then slowly drive forward (away from us) for 5 seconds.
        // Finally idle for the rest of auton
        drivetrain.applyRequest(() -> idle));
  }
}
