package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Grams;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
import frc.robot.util.BallConstants;
import frc.robot.util.BallPhysics;
import frc.robot.util.BallSimulator;
import frc.robot.util.BallState;
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
    private int counter = 0;
    private Pose3d effectiveTargetLocation;
    private ShotSolution shotSolution;
    private BallSimulator ballSimulator = new BallSimulator(new BallConstants(
            Grams.of(210).in(Kilograms), Inches.of(3).in(Meters), 1.2, 0.30, 1.2, 0.35, 9.81, 20), FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH);

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
        nt.putValue("is_aligned", NetworkTableValue.makeBoolean(isAngleWithinTolerance()));
        otfSolutionPublisher.set(effectiveTargetLocation);

        // counter++;
        // if (counter > 10) {
        //     counter = 0;
        //      Translation3d drivetrainVeloTransform = new Translation3d(drivetrain.getChassisSpeeds().vxMetersPerSecond,
        //         drivetrain.getChassisSpeeds().vyMetersPerSecond, 0);
        //         Pose2d drivetrainPose = drivetrain.getPose2d();
        //     ballSimulator.addBall(new BallState(
        //         new Pose3d(new Translation3d(drivetrainPose.getX(), drivetrainPose.getY(), 0),
        //                         new Rotation3d(drivetrainPose.getRotation())),
        //         new Translation3d(
        //                         shotSolution.launchSpeed() * Math.cos(shotSolution.launchPitchRad()),
        //                         0,
        //                         shotSolution.launchSpeed() * Math.sin(shotSolution.launchPitchRad()))
        //                         .rotateBy(new Rotation3d(drivetrainPose.getRotation()))
        //                         .plus(drivetrainVeloTransform),
        //                 new Translation3d(0, 100, 0)));
        // }
        // ballSimulator.update();
    }

    public void solveOTF() {
        ChassisSpeeds speeds = drivetrain.getChassisSpeeds();
        Pose3d goal = FieldConstants.getHubPosition().plus(new Transform3d(0d, 0d, -Units.inchesToMeters(18), new Rotation3d()));

        Pose2d currentPose = drivetrain.getPose2d();

        effectiveTargetLocation = ShootOnTheFlyCalculator.calculateEffectiveTargetLocation(
                currentPose,
                goal, speeds, accel,
                (dist) -> {return ShooterConstants.DISTANCE_TO_SHOOT_VELOCITY.get(dist);},
                5, 0

        );    

        if(currentPose.getX() - goal.getX() < 0) {
            effectiveTargetLocation = rotate180(effectiveTargetLocation);
        }

        double targetDistance = currentPose.getTranslation().getDistance(effectiveTargetLocation.toPose2d().getTranslation());
        nt.putValue("target_distance", NetworkTableValue.makeDouble(targetDistance));

        shotSolution = BallPhysics.solveBallisticWithIncomingAngle(new Pose3d(currentPose), effectiveTargetLocation, Units.degreesToRadians(ShooterConstants.INCOMMING_SHOT_ANGLE));

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

        return new Rotation2d(MathUtil.angleModulus(Math.atan2(offsetY, offsetX)) + (FieldConstants.getAlliance() == Alliance.Blue ? Math.PI : 0
        ));
    }

    public boolean isAngleWithinTolerance() {
        double angleOffset = Math.abs(getAimAngle().getDegrees() - drivetrain.getPose2d().getRotation().getDegrees());
        nt.putValue("Angle Offset", NetworkTableValue.makeDouble(angleOffset));
        return angleOffset < 3.0 || Math.abs(180 - angleOffset) < 3.0;
    }

    public boolean isOTFSolution() {
        return !effectiveTargetLocation.equals(new Pose3d());
    }

    public boolean isShotSolution() {
        return !shotSolution.equals(new ShotSolution(0, 0, 0));
    }

    private Pose3d rotate180(Pose3d pose3d) {
        return pose3d.rotateAround(FieldConstants.getHubPosition().getTranslation(), new Rotation3d(0,0,Math.PI));
    }

}
