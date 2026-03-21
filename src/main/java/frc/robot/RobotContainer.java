// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.KickerConstants;
import frc.robot.Constants.MidtakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElasticSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.MidtakeSubsystem;
import frc.robot.subsystems.ShootOnTheFlyCalculatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
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
        private final SendableChooser<Command> autoChooser;
        private final CommandXboxController driveController = new CommandXboxController(
                        ControllerConstants.DRIVER_CONTROLLER_PORT);
        private final CommandXboxController operatorController = new CommandXboxController(
                        ControllerConstants.OPERATOR_CONTROLLER_PORT);

        // ---------- CTRE Drivetrain ----------\\
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final Telemetry logger = new Telemetry(DriveConstants.MAX_ROBOT_VELOCITY);

        // ---------- Subsystems ----------\\
        private final ShooterSubsystem shooter = new ShooterSubsystem();
        private final KickerSubsystem kickerSubsystem = new KickerSubsystem();
        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        private final MidtakeSubsystem MTKESubsystem = new MidtakeSubsystem();
        private final ShootOnTheFlyCalculatorSubsystem otfSubsystem = new ShootOnTheFlyCalculatorSubsystem(drivetrain);
        public final DriveSubsystem driveSubsystem = new DriveSubsystem(drivetrain, driveController.getHID());
        private final ElasticSubsystem elasticSubsystem = new ElasticSubsystem();

        // ---------- Swerve Requests ----------\\
        private final SwerveRequest.Idle idle = new SwerveRequest.Idle();
        private final SwerveRequest.SwerveDriveBrake xStance = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(DriveConstants.MAX_ROBOT_VELOCITY * 0.01)
                        .withRotationalDeadband(DriveConstants.MAX_ROBOT_RAD_VELOCITY * 0.01)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final SwerveRequest.FieldCentricFacingAngle aimAtHub = new SwerveRequest.FieldCentricFacingAngle()
                        .withHeadingPID(DriveConstants.ROTATION_CONTROLLER.getP(),
                                        DriveConstants.ROTATION_CONTROLLER.getI(),
                                        DriveConstants.ROTATION_CONTROLLER.getD());

        public RobotContainer() {
                PhotonCameraContainer.addPhotonCamera(
                                new VisionCamera.Builder().withName("FRcamera")
                                                .withCameraEnabled()
                                                .withSingleTagEstimation()
                                                .withOffset(
                                                                new Transform3d(
                                                                                Units.inchesToMeters(-10),
                                                                                Units.inchesToMeters(10),
                                                                                0,
                                                                                new Rotation3d(
                                                                                                0,
                                                                                                Units.degreesToRadians(
                                                                                                                25),
                                                                                                Units.degreesToRadians(
                                                                                                                40))))
                                                .build());

                PhotonCameraContainer.addPhotonCamera(
                                new VisionCamera.Builder().withName("FLcamera")
                                                .withCameraEnabled()
                                                .withSingleTagEstimation()
                                                .withOffset(
                                                                new Transform3d(
                                                                                -Units.inchesToMeters(10),
                                                                                -Units.inchesToMeters(10),
                                                                                0,
                                                                                new Rotation3d(
                                                                                                0,
                                                                                                Units.degreesToRadians(
                                                                                                                25),
                                                                                                Units.degreesToRadians(
                                                                                                                -40))))
                                                .build());

                // ---------- Commands ----------\\
                NamedCommands.registerCommand("Startup", new InstantCommand(() -> {
                        intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
                        MTKESubsystem.setMidtakeSpeed(0);
                        kickerSubsystem.setKickerSpeed(0);
                        shooter.setShooterRPM(1500);
                        shooter.setTargetHoodAngle(ShooterConstants.MIN_HOOD_ANGLE);
                        drivetrain.setDefaultCommand(null);
                }));

                NamedCommands.registerCommand("Find Camera", new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                        if (PhotonCameraContainer.HAS_MULTI_TAG_ESTIMATE == false) {
                                                drivetrain.applyRequest(getRotateSlow()).execute();
                                        }
                                }),
                                new WaitUntilCommand(() -> {
                                        if (PhotonCameraContainer.HAS_MULTI_TAG_ESTIMATE) {
                                                drivetrain.applyRequest(() -> xStance).execute();
                                                return true;
                                        }

                                        return false;
                                })));

                NamedCommands.registerCommand("Shoot", new ParallelCommandGroup(
                                new RepeatCommand(drivetrain.applyRequest(getAimRequest())),
                                new WaitCommand(0.25).andThen(new InstantCommand(() -> {
                                        MTKESubsystem.setMidtakeSpeed(
                                                        MidtakeConstants.MIDTAKE_SPEED);
                                        kickerSubsystem.setKickerSpeed(
                                                        KickerConstants.KICKER_SPEED);
                                })),
                                new ShootCommand(shooter, otfSubsystem)).withDeadline(new WaitCommand(4 )));

                NamedCommands.registerCommand("Stop Shoot", new InstantCommand(() -> {
                        MTKESubsystem.setMidtakeSpeed(0);
                        kickerSubsystem.setKickerSpeed(0);
                        shooter.setShooterRPM(1500);
                        shooter.setTargetHoodAngle(ShooterConstants.MIN_HOOD_ANGLE);
                }));

                NamedCommands.registerCommand("Intake Out", new InstantCommand(() -> {
                        intakeSubsystem.setIntakePosition(IntakePosition.DEPLOYED);
                        intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
                }));

                NamedCommands.registerCommand("Intake In", new InstantCommand(() -> {
                        intakeSubsystem.setIntakePosition(IntakePosition.STOWED);
                        intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
                }));

                NamedCommands.registerCommand("Intake In", new InstantCommand(() -> {
                        intakeSubsystem.setIntakePosition(IntakePosition.STOWED);
                        intakeSubsystem.setIntakeSpeed(0);
                }));

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);

                configureBindings();
        }

        private Supplier<SwerveRequest> getRotateSlow() {
                return () -> drive
                                .withVelocityX(0)
                                .withVelocityY(0)
                                .withRotationalRate(0.1 * DriveConstants.MAX_ROBOT_RAD_VELOCITY);
        }

        public void autonomousPeriodic() {
                PhotonCameraContainer.estimateVisionOdometry(drivetrain);
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

                driveController.pov(0).onTrue(new InstantCommand(() -> {
                        intakeSubsystem.setIntakePosition(IntakePosition.STOWED);
                }));
                driveController.pov(180).onTrue(new InstantCommand(() -> {
                        intakeSubsystem.setIntakePosition(IntakePosition.DEPLOYED);
                }));

                // ---------- Operator Controller ----------\\

                operatorController.rightBumper().onTrue(new InstantCommand(() -> {
                        intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
                })).onFalse(new InstantCommand(() -> {
                        intakeSubsystem.setIntakeSpeed(0);
                }));

                operatorController.button(8).onTrue(new InstantCommand(() -> {
                        MTKESubsystem.setMidtakeSpeed(MidtakeConstants.MIDTAKE_SPEED);
                })).onFalse(new InstantCommand(() -> {
                        MTKESubsystem.setMidtakeSpeed(0);
                }));

                operatorController.leftTrigger().onTrue(new InstantCommand(() -> {
                        kickerSubsystem.setKickerSpeed(KickerConstants.KICKER_SPEED);
                        MTKESubsystem.setMidtakeSpeed(MidtakeConstants.MIDTAKE_SPEED);
                        shooter.setShooterRPM(3000);
                        shooter.setTargetHoodAngle(ShooterConstants.MIN_HOOD_ANGLE + 20);

                })).onFalse(new InstantCommand(() -> {
                        kickerSubsystem.setKickerSpeed(0);
                        MTKESubsystem.setMidtakeSpeed(0);
                        shooter.setShooterRPM(1000);
                        shooter.setTargetHoodAngle(ShooterConstants.MIN_HOOD_ANGLE);
                }));

                operatorController.b().onTrue(new InstantCommand(() -> {
                        MTKESubsystem.setMidtakeSpeed(-MidtakeConstants.MIDTAKE_SPEED);
                })).onFalse(new InstantCommand(() -> {
                        MTKESubsystem.setMidtakeSpeed(0);
                }));

                operatorController.leftBumper().onTrue(new InstantCommand(() -> {
                        kickerSubsystem.setKickerSpeed(KickerConstants.KICKER_SPEED);
                        MTKESubsystem.setMidtakeSpeed(MidtakeConstants.MIDTAKE_SPEED);
                        shooter.setShooterRPM(3500);
                        shooter.setTargetHoodAngle(ShooterConstants.MIN_HOOD_ANGLE);
                        intakeSubsystem.setShake(true);
                })).onFalse(new InstantCommand(() -> {
                        kickerSubsystem.setKickerSpeed(0);
                        MTKESubsystem.setMidtakeSpeed(0);
                        shooter.setShooterRPM(1000);
                        shooter.setTargetHoodAngle(ShooterConstants.MIN_HOOD_ANGLE);
                        intakeSubsystem.setShake(false);
                }));
                operatorController.x().onTrue(new SequentialCommandGroup(
                                new WaitUntilCommand(() -> {
                                        shooter.setTargetHoodAngle(ShooterConstants.MIN_HOOD_ANGLE);
                                        shooter.getHoodMotor().setVoltage(-1);
                                        SmartDashboard.putNumber("hood_current",
                                                        shooter.getHoodMotor().getOutputCurrent());
                                        return Math.abs(shooter.getHoodMotor().getOutputCurrent()) > 22;
                                }),
                                new InstantCommand(() -> {
                                        System.out.print("Reset Hood Angle!");
                                        shooter.getHoodMotor().setVoltage(0);
                                        shooter.getHoodMotor().getEncoder()
                                                        .setPosition(ShooterConstants.MIN_HOOD_ANGLE / 360.0);
                                })));
                operatorController.rightTrigger()
                                .whileTrue(new ParallelCommandGroup(
                                                new RepeatCommand(drivetrain.applyRequest(getAimRequest())),
                                                new RepeatCommand(
                                                                new ParallelCommandGroup(
                                                                                new WaitCommand(1).andThen(
                                                                                                new InstantCommand(
                                                                                                                () -> {
                                                                                                                        MTKESubsystem.setMidtakeSpeed(
                                                                                                                                        MidtakeConstants.MIDTAKE_SPEED);
                                                                                                                        kickerSubsystem.setKickerSpeed(
                                                                                                                                        KickerConstants.KICKER_SPEED);
                                                                                                                })),
                                                                                new ShootCommand(
                                                                                                shooter,
                                                                                                otfSubsystem)))))
                                .onFalse(new InstantCommand(() -> {
                                        drivetrain.applyRequest(getDefaultDriveCommand());
                                        MTKESubsystem.setMidtakeSpeed(0);
                                        kickerSubsystem.setKickerSpeed(0);
                                        shooter.setShooterRPM(1000);
                                        shooter.setTargetHoodAngle(ShooterConstants.MIN_HOOD_ANGLE);
                                }));

                RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));
                drivetrain.registerTelemetry(logger::telemeterize);

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public Pose2d getStartingPose() {
                if (PhotonCameraContainer.HAS_MULTI_TAG_ESTIMATE) {
                        return drivetrain.getPose2d();
                } else {
                        // Assume against alliance wall (hub center)
                        Alliance alliance = FieldConstants.getAlliance();
                        double x;
                        Rotation2d rotation;
                        if (alliance == Alliance.Red) {
                                x = FieldConstants.FIELD_LENGTH
                                                - (frc.robot.Constants.RobotConstants.robotLengthMeters / 2.0);
                                rotation = Rotation2d.fromDegrees(180);
                        } else {
                                x = frc.robot.Constants.RobotConstants.robotLengthMeters / 2.0;
                                rotation = Rotation2d.fromDegrees(0);
                        }
                        return new Pose2d(x, FieldConstants.FIELD_WIDTH / 2.0, rotation);
                }
        }

        public void onTeleOP() {
                MTKESubsystem.setMidtakeSpeed(0);
                kickerSubsystem.setKickerSpeed(0);
                shooter.setShooterRPM(1000);
                shooter.setTargetHoodAngle(ShooterConstants.MIN_HOOD_ANGLE);
                drivetrain.setDefaultCommand(drivetrain.applyRequest(getDefaultDriveCommand()));
        }

        private Supplier<SwerveRequest> getDefaultDriveCommand() {
                return () -> drive
                                .withVelocityX(driveController.getLeftY() * driveSubsystem.getSlewRateMultiplier()
                                                * DriveConstants.MAX_ROBOT_VELOCITY
                                                * -1)
                                .withVelocityY(driveController.getLeftX() * driveSubsystem.getSlewRateMultiplier()
                                                * DriveConstants.MAX_ROBOT_VELOCITY
                                                * -1)
                                .withRotationalRate(
                                                -driveController.getRightX() * driveSubsystem.getSlewRateMultiplier()
                                                                * DriveConstants.MAX_ROBOT_RAD_VELOCITY);
        }

        private Supplier<SwerveRequest> getAimRequest() {
                return () -> aimAtHub.withTargetDirection(otfSubsystem.getAimAngle())
                                .withVelocityX(driveController.getLeftY() * driveSubsystem.getSlewRateMultiplier()
                                                * DriveConstants.MAX_ROBOT_VELOCITY
                                                * -1)
                                .withVelocityY(driveController.getLeftX() * driveSubsystem.getSlewRateMultiplier()
                                                * DriveConstants.MAX_ROBOT_VELOCITY
                                                * -1)
                                .withMaxAbsRotationalRate(0.25 * DriveConstants.MAX_ROBOT_RAD_VELOCITY);

        }
}
