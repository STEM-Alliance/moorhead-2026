package frc.robot.util;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
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
            .withDeadband(DriveConstants.MAX_ROBOT_VELOCITY * 0.1).withRotationalDeadband(DriveConstants.MAX_ROBOT_RAD_VELOCITY * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake xStance = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    public DriveStateHandler(CommandSwerveDrivetrain commandSwerveDrivetrain, XboxController xbox) {
        this.xbox = xbox;
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;

        requestedDriveCommand = idle;

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

        switch (commandSwerveDrivetrain.getRotationStyle()) {
            case Driver -> {
                Translation2d xyRaw = new Translation2d(xbox.getLeftX(), xbox.getLeftY());
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
                // forward, left, rotate
                if (commandSwerveDrivetrain.getDriveState() == DriveState.Free) {
                    requestedDriveCommand = drive.withVelocityX(xSpeed)
                            .withVelocityY(ySpeed)
                            .withRotationalRate(zSpeed);
                }

            }
            case Aimbot -> {

            }

            case Climb -> {

            }
        }

        return requestedDriveCommand;
    }
}
