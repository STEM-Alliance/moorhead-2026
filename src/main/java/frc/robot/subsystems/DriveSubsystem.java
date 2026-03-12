package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathPlannerConstants;

public class DriveSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final XboxController driverXboxController;

    private final SlewRateLimiter dsratelimiter = new SlewRateLimiter(4);

    private double driveMultiplier = 0.0;

    private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("drivetrain");

    private final StructPublisher<Pose2d> robotPosePublisher = nt.getStructTopic("robot_pose", Pose2d.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> moduleTargetsPublisher = nt
            .getStructArrayTopic("swerve_targets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> moduleStatePublisher = nt
            .getStructArrayTopic("swerve_states", SwerveModuleState.struct).publish();
    private final Field2d robotField = new Field2d();
    private final SwerveRequest.FieldCentric autoDrive = new SwerveRequest.FieldCentric()
                        .withDeadband(DriveConstants.MAX_ROBOT_VELOCITY * 0.1)
                        .withRotationalDeadband(DriveConstants.MAX_ROBOT_RAD_VELOCITY * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public DriveSubsystem(CommandSwerveDrivetrain drivetrain, XboxController driverXboxController) {
        this.drivetrain = drivetrain;
        this.driverXboxController = driverXboxController;

    }

    public void setChassisSpeedsAUTO(ChassisSpeeds speeds) {
        drivetrain.applyRequest(getRequest(speeds));
    }

    private Supplier<SwerveRequest> getRequest(ChassisSpeeds speeds) {
                return () -> autoDrive
                                .withVelocityX(speeds.vxMetersPerSecond * DriveConstants.SLOW_MODE_MULT
                                                * DriveConstants.MAX_ROBOT_VELOCITY
                                                * (FieldConstants.getAlliance() == Alliance.Blue ? -1 : 1))
                                .withVelocityY(speeds.vyMetersPerSecond *  DriveConstants.SLOW_MODE_MULT
                                                * DriveConstants.MAX_ROBOT_VELOCITY
                                                * (FieldConstants.getAlliance() == Alliance.Blue ? -1 : 1))
                                .withRotationalRate(
                                               speeds.omegaRadiansPerSecond *  DriveConstants.SLOW_MODE_MULT
                                            );
        }

    @Override
    public void periodic() {
        driveMultiplier = dsratelimiter.calculate((DriveConstants.FULL_DRIVE_MODE_MULT - DriveConstants.SLOW_MODE_MULT)
                * driverXboxController.getRightTriggerAxis() + DriveConstants.SLOW_MODE_MULT);
        robotField.setRobotPose(drivetrain.getPose2d());

        SmartDashboard.putData("elastic_field_pose", robotField);
        robotPosePublisher.set(drivetrain.getPose2d());
        moduleStatePublisher.set(drivetrain.getState().ModuleStates);
        moduleTargetsPublisher.set(drivetrain.getState().ModuleTargets);

    }

    public double getSlewRateMultiplier() {
        return driveMultiplier;
    }
}
