// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShootOnTheFlyCalculatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.PhotonCameraContainer;
import frc.robot.util.Telemetry;
import frc.robot.util.TunerConstants;
import frc.robot.util.VisionCamera;

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

    private final CommandXboxController driveController = new CommandXboxController(
            ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(
            ControllerConstants.OPERATOR_CONTROLLER_PORT);

    // ---------- CTRE Drivetrain ----------\\
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Telemetry logger = new Telemetry(DriveConstants.MAX_ROBOT_VELOCITY);

    // ---------- Subsystems ----------\\
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final ShootOnTheFlyCalculatorSubsystem otfSubsystem = new ShootOnTheFlyCalculatorSubsystem(drivetrain);
    public final DriveSubsystem driveSubsystem = new DriveSubsystem(drivetrain, driveController.getHID());

    // ---------- Swerve Requests ----------\\
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();
    private final SwerveRequest.SwerveDriveBrake xStance = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MAX_ROBOT_VELOCITY * 0.1)
            .withRotationalDeadband(DriveConstants.MAX_ROBOT_RAD_VELOCITY * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentricFacingAngle aimAtHub = new SwerveRequest.FieldCentricFacingAngle()
            .withHeadingPID(DriveConstants.ROTATION_CONTROLLER.getP(), DriveConstants.ROTATION_CONTROLLER.getI(),
                    DriveConstants.ROTATION_CONTROLLER.getD());

    public RobotContainer() {
        PhotonCameraContainer.addPhotonCamera(
                new VisionCamera.Builder().withName("driveCamera")
                        .withCameraEnabled()
                        .withSingleTagEstimation()
                        .build());

        drivetrain.setDefaultCommand(drivetrain.applyRequest(getDefaultDriveCommand()));

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

        driveController.button(8).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        driveController.b().onTrue(drivetrain.applyRequest(() -> xStance));

        driveController.rightBumper()
                .whileTrue(new ParallelCommandGroup(
                        new RepeatCommand(drivetrain.applyRequest(getAimRequest())),
                        new RepeatCommand(new SequentialCommandGroup(
                                new WaitUntilCommand(
                                        new BooleanSupplier() {
                                            @Override
                                            public boolean getAsBoolean() {
                                                return RobotBase.isReal()
                                                        ? shooter.isReadyToShoot()
                                                                && otfSubsystem.isAngleWithinTolerance()
                                                        : true;
                                            }
                                        }),
                                new ShootCommand(shooter),
                                // Add prep commands for shooting here...
                                new PrintCommand("Shooting!")))))
                .onFalse(drivetrain.applyRequest(getDefaultDriveCommand()));

        RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));
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

    private Supplier<SwerveRequest> getDefaultDriveCommand() {
        return () -> drive
                .withVelocityX(-driveController.getLeftY() * driveSubsystem.getSlewRateMultiplier()
                        * DriveConstants.MAX_ROBOT_VELOCITY)
                .withVelocityY(-driveController.getLeftX() * driveSubsystem.getSlewRateMultiplier()
                        * DriveConstants.MAX_ROBOT_VELOCITY)
                .withRotationalRate(-driveController.getRightX() * driveSubsystem.getSlewRateMultiplier()
                        * DriveConstants.MAX_ROBOT_RAD_VELOCITY);
    }

    private Supplier<SwerveRequest> getAimRequest() {
        return () -> aimAtHub.withTargetDirection(otfSubsystem.getAimAngle())
                .withVelocityX(-driveController.getLeftY() * driveSubsystem.getSlewRateMultiplier()
                        * DriveConstants.MAX_ROBOT_VELOCITY)
                .withVelocityY(-driveController.getLeftX() * driveSubsystem.getSlewRateMultiplier()
                        * DriveConstants.MAX_ROBOT_VELOCITY)
                .withMaxAbsRotationalRate(0.25 * DriveConstants.MAX_ROBOT_RAD_VELOCITY);

    }
}
