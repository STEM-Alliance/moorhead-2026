package frc.robot.util;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.Rotation;

import java.util.Optional;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveState;

public class DriveStateHandler {
    private final XboxController xbox;
    private SlewRateLimiter dsratelimiter = new SlewRateLimiter(4);
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;
    private double DRIVE_MULT = 1.0;
    private final double SLOWMODE_MULT = 0.25;

    private SwerveRequest requestedDriveCommand;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MAX_ROBOT_VELOCITY * 0.01).withRotationalDeadband(DriveConstants.MAX_ROBOT_RAD_VELOCITY * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake xStance = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    public DriveStateHandler(CommandSwerveDrivetrain commandSwerveDrivetrain, XboxController xbox) {
        this.xbox = xbox;
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        requestedDriveCommand = idle;

        DriveConstants.AIMBOT_CONTROLLER.enableContinuousInput(0, 2 * Math.PI);

    }

    public Translation2d deadBand(Translation2d input, double deadzone) {
        double mag = input.getNorm();
        Translation2d norm = input.div(mag);

        if (mag < deadzone) {
            return new Translation2d(0.0, 0.0);
        } else {
            // TODO: Check is it sqrt2 or 1.0...
            Translation2d result = norm.times((mag - deadzone) / (1.0 - deadzone));
            return new Translation2d(
                    MathUtil.clamp(result.getX(), -1.0, 1.0),
                    MathUtil.clamp(result.getY(), -1.0, 1.0));
        }
    }

    public double deadBand(double input, double deadband) {
        return Math.abs(input) < deadband ? 0.0 : (input - Math.signum(input) * deadband) / (1.0 - deadband);
    }
    public SwerveRequest getSwerveRequest() {
        switch (commandSwerveDrivetrain.getDriveState()) {
            case Free -> {

            }
            case Locked -> {
                requestedDriveCommand = xStance;
            }
        }
 Translation2d xyRaw = new Translation2d(xbox.getLeftY(), -xbox.getLeftX());
                Translation2d xySpeed = deadBand(xyRaw, 0.15 / 2.f);
                double zSpeed = -deadBand(xbox.getRightX(), 0.15);
                double xSpeed = -xySpeed.getX(); // xbox.getLeftX();
                double ySpeed = xySpeed.getY(); // xbox.getLeftY();

                // System.out.println("DRIVE!!");

                // double mag_xy = Math.sqrt(xSpeed*xSpeed + ySpeed*ySpeed);

                // xSpeed = mag_xy > 0.15 ? xSpeed : 0.0;
                // ySpeed = mag_xy > 0.15 ? ySpeed : 0.0;
                // zSpeed = Math.abs(zSpeed) > 0.15 ? zSpeed : 0.0;

                xSpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
                ySpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
                zSpeed *= DriveConstants.Z_SPEED_LIMIT * DriveConstants.MAX_ROBOT_RAD_VELOCITY;
                double dmult = dsratelimiter
                        .calculate((DRIVE_MULT - SLOWMODE_MULT) * xbox.getRightTriggerAxis() + SLOWMODE_MULT);
                xSpeed *= dmult;
                ySpeed *= dmult;
                zSpeed *= dmult;
        switch (commandSwerveDrivetrain.getRotationStyle()) {
            case Driver -> {
               
                // forward, left, rotate
                if (commandSwerveDrivetrain.getDriveState() == DriveState.Free) {
                    requestedDriveCommand = drive.withVelocityX(xSpeed)
                            .withVelocityY(ySpeed)
                            .withRotationalRate(zSpeed);
                }

            }
            case Aimbot -> {
                Pose2d hubPose = FieldConstants.getHubPosition().toPose2d();
                Optional<Pose2d> robotPoseOpt = commandSwerveDrivetrain.samplePoseAt(Utils.getCurrentTimeSeconds());
                if (robotPoseOpt.isPresent()) {
                    Pose2d robotPose = robotPoseOpt.get();
                    double targetOffset = hubPose.rotateAround(robotPose.getTranslation(), robotPose.getRotation()).getRotation().getRadians();
                    SmartDashboard.putNumber("Robot Rotation", robotPose.getRotation().getRadians());
                    SmartDashboard.putNumber("Target Offset", targetOffset);

                    // RelativeTo's rotation returns robot rotation

                    double offsetY = hubPose.getY() - robotPose.getY(); // Short Side
                    double offsetX = hubPose.getX() - robotPose.getX(); // Long Side

                    double goalDegrees =  MathUtil.angleModulus(Math.tan(offsetY / offsetX) + Math.PI);
                    double goalZSpeed = DriveConstants.AIMBOT_CONTROLLER.calculate(robotPose.getRotation().getRadians(), targetOffset);
                    SmartDashboard.putNumber("OffsetDegree",goalDegrees);
                    SmartDashboard.putNumber("Robot Offset X", offsetX);
                    SmartDashboard.putNumber("Robot Offset Y", offsetY);
                    SmartDashboard.putNumber("ZSpeed", MathUtil.angleModulus(goalZSpeed));
                    // requestedDriveCommand = drive.withVelocityX(xSpeed)
                    //         .withVelocityY(ySpeed)
                    //         .withRotationalRate(goalZSpeed);
                }
            }

            case Climb -> {

            }
        }

        return requestedDriveCommand;
    }
}
