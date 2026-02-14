package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.BallPhysics;
import frc.robot.util.ChassisAccelerations;
import frc.robot.util.ShootOnTheFlyCalculator;
import frc.robot.util.BallPhysics.ShotSolution;
import frc.robot.util.ShootOnTheFlyCalculator.InterceptSolution;

public class ShootOnTheFlyCalculatorSubsystem extends SubsystemBase {
    private ChassisSpeeds prevSpeeds = new ChassisSpeeds();
    private ChassisAccelerations accel = new ChassisAccelerations(0, 0, 0);
    private InterceptSolution interceptSolution;
    private final CommandSwerveDrivetrain drivetrain;

    private double prevTimestamp = 0.0;
    private double dt = 0.02;
    private Pose3d effectiveTargetLocation;
    private ShotSolution shotSolution;

    private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("otf_calculator");

    StructPublisher<Pose3d> otfSolutionPublisher = nt.getStructTopic("otf_solution", Pose3d.struct).publish();

    public ShootOnTheFlyCalculatorSubsystem(CommandSwerveDrivetrain commandSwerveDrivetrain) {
        this.drivetrain = commandSwerveDrivetrain;
        this.effectiveTargetLocation = new Pose3d();
        this.shotSolution = new ShotSolution(0,0,0);
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
        nt.putValue("shot_pitch", NetworkTableValue.makeDouble(Units.radiansToDegrees(shotSolution.launchPitchRad())));
        nt.putValue("shot_speed", NetworkTableValue.makeDouble(shotSolution.launchSpeed()));
        nt.putValue("is_shot_solution", NetworkTableValue.makeBoolean(isShotSolution()));
        otfSolutionPublisher.set(effectiveTargetLocation);
    }

    public void solveOTF() {
        ChassisSpeeds speeds = drivetrain.getChassisSpeeds();
        Pose3d goal = FieldConstants.getHubPosition();

        Pose2d currentPose = drivetrain.getPose2d();

        effectiveTargetLocation = ShootOnTheFlyCalculator.calculateEffectiveTargetLocation(
                currentPose,
                goal, speeds, accel,
                (dist) -> {return ShooterConstants.DISTANCE_TO_SHOOT_VELOCITY.get(dist);},
                5, 0

        );    

        if(currentPose.getX() - goal.getX() > 0) {
            effectiveTargetLocation = rotate180(effectiveTargetLocation);
        }
        double targetDistance = currentPose.getTranslation().getDistance(effectiveTargetLocation.toPose2d().getTranslation());

        shotSolution = BallPhysics.solveBallisticWithIncomingAngle(new Pose3d(currentPose), effectiveTargetLocation, Units.degreesToRadians(1));

    }

    public Pose3d getOTFSolution() {
        return effectiveTargetLocation;
    }

    public ShotSolution getShotSolution() {
        return shotSolution;
    }

    public Rotation2d getAimAngle() {
        Pose2d targetPose;
        if (isOTFSolution()) {
            targetPose = getOTFSolution().toPose2d();
        } else {
            targetPose = FieldConstants.getHubPosition().toPose2d();
        }
        
        Pose2d robotPose = drivetrain.getPose2d();
        double offsetX = robotPose.getX() - targetPose.getX(); // Long Side
        double offsetY = robotPose.getY() - targetPose.getY(); // Short Side

        return new Rotation2d(MathUtil.angleModulus(Math.atan2(offsetY, offsetX)));
    }

    public boolean isAngleWithinTolerance() {
        return Math.abs(getAimAngle().getDegrees() - drivetrain.getPose2d().getRotation().getDegrees()) < 5.0;
    }

    public boolean isOTFSolution() {
        return effectiveTargetLocation != new Pose3d();
    }

    public boolean isShotSolution() {
        return shotSolution.equals(new ShotSolution(0, 0, 0));
    }

    private Pose3d rotate180(Pose3d pose3d) {
        return pose3d.rotateAround(FieldConstants.getHubPosition().getTranslation(), new Rotation3d(0,0,Math.PI));
    }

}
