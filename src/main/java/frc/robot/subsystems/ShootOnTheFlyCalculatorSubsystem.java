package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.ChassisAccelerations;
import frc.robot.util.ShootOnTheFlyCalculator;
import frc.robot.util.ShootOnTheFlyCalculator.InterceptSolution;

public class ShootOnTheFlyCalculatorSubsystem extends SubsystemBase {
    private ChassisSpeeds prevSpeeds = new ChassisSpeeds();
    private ChassisAccelerations accel = new ChassisAccelerations(0, 0, 0);
    private InterceptSolution interceptSolution;
    private final CommandSwerveDrivetrain drivetrain;

    private double prevTimestamp = 0.0;
    private double dt = 0.02;

    private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("otf_calculator");

    StructPublisher<Pose3d> otfSolutionPublisher = nt.getStructTopic("otf_solution", Pose3d.struct).publish();

    public ShootOnTheFlyCalculatorSubsystem(CommandSwerveDrivetrain commandSwerveDrivetrain) {
        this.drivetrain = commandSwerveDrivetrain;
    }

    @Override
    public void periodic() {
        ChassisSpeeds speeds = drivetrain.getChassisSpeeds();
        prevTimestamp = Timer.getFPGATimestamp();

        accel = new ChassisAccelerations(speeds, prevSpeeds, dt);

        solveOTF();

        prevSpeeds = speeds;
        dt = Timer.getFPGATimestamp() - prevTimestamp;

        nt.putValue("is_otf_solution", NetworkTableValue.makeBoolean(isOTFSolution()));
        otfSolutionPublisher.set(interceptSolution.effectiveTargetPose());
    }

    public void solveOTF() {
        ChassisSpeeds speeds = drivetrain.getChassisSpeeds();
        Pose3d goal = FieldConstants.getHubPosition();

        Pose2d currentPose = drivetrain.getPose2d();
        double targetDistance = currentPose.getTranslation().getDistance(goal.toPose2d().getTranslation());

        interceptSolution = ShootOnTheFlyCalculator.solveShootOnTheFly(
                toPose3d(currentPose),
                goal, speeds, accel,
                ShooterConstants.DISTANCE_TO_SHOT_SPEED.get(targetDistance),
                5, 0.01

        );

    }

    public InterceptSolution getOTFSolution() {
        return interceptSolution;
    }

    public Rotation2d getAimAngle() {
        Pose2d targetPose;
        if (isOTFSolution()) {
            targetPose = getOTFSolution().effectiveTargetPose().toPose2d();
        } else {
            targetPose = FieldConstants.getHubPosition().toPose2d();
        }

        Pose2d robotPose = drivetrain.getPose2d();
        Transform2d delta = targetPose.minus(robotPose);

        return new Rotation2d(Math.atan2(delta.getY(), delta.getX()));
    }

    public boolean isAngleWithinTolerance() {
        return Math.abs(getAimAngle().getDegrees() - drivetrain.getPose2d().getRotation().getDegrees()) < 5.0;
    }

    public boolean isOTFSolution() {
        return interceptSolution != null;
    }

    private Pose3d toPose3d(Pose2d pose2d) {
        return new Pose3d(pose2d.getX(), pose2d.getY(), 0.0, new Rotation3d(pose2d.getRotation()));
    }

}
