package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LEDSubsystem.LedStyle;
import frc.robot.util.*;
import frc.robot.util.BallPhysics.ShotSolution;

import static edu.wpi.first.units.Units.*;

public class ShootOnTheFlyCalculatorSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final LEDSubsystem ledSubsystem;
    private final DigitalInput beamBreak;

    private ChassisSpeeds prevSpeeds = new ChassisSpeeds();
    private ChassisAccelerations accel = new ChassisAccelerations(0, 0, 0);
    private boolean wasBeamBreakBroken = false;
    private boolean shouldAimAtShuttlePoints = false;
    private int ballsShot = 0;

    private double prevTimestamp = 0.0;
    private double deltaTime = 0.02;
    private Pose3d effectiveTargetLocation;
    private ShotSolution shotSolution;
    private BallSimulator ballSimulator = new BallSimulator(new BallConstants(
            Grams.of(210).in(Kilograms), Inches.of(3).in(Meters), 1.2, 0.30, 1.2, 0.35, 9.81, 20),
            FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH);
    private Translation2d aimAtPoint = Translation2d.kZero;

    private final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("otf_calculator");
    private final StructArrayPublisher<Pose3d> goalAimPosition = networkTable
            .getStructArrayTopic("aim_at_points", Pose3d.struct)
            .publish();

    StructPublisher<Pose3d> otfSolutionPublisher = networkTable.getStructTopic("otf_solution", Pose3d.struct).publish();

    public ShootOnTheFlyCalculatorSubsystem(CommandSwerveDrivetrain commandSwerveDrivetrain,
            LEDSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        beamBreak = new DigitalInput(ShooterConstants.BEAM_BREAK_PORT);

        this.drivetrain = commandSwerveDrivetrain;
        this.effectiveTargetLocation = new Pose3d();
        this.shotSolution = new ShotSolution(0, 0, 0);
    }

    @Override
    public void periodic() {
        ChassisSpeeds speeds = drivetrain.getChassisSpeeds();
        prevTimestamp = Timer.getFPGATimestamp();

        accel = new ChassisAccelerations(speeds, prevSpeeds, deltaTime);

        solveOTF();

        prevSpeeds = speeds;
        deltaTime = Timer.getFPGATimestamp() - prevTimestamp;

        networkTable.putValue("is_otf_solution", NetworkTableValue.makeBoolean(isOTFSolution()));
        networkTable.putValue("shot_pitch",
                NetworkTableValue.makeDouble(Units.radiansToDegrees(shotSolution.launchPitchRad())));
        networkTable.putValue("shot_speed", NetworkTableValue.makeDouble(shotSolution.launchSpeed()));
        networkTable.putValue("is_shot_solution", NetworkTableValue.makeBoolean(isShotSolution()));
        networkTable.putValue("is_aligned", NetworkTableValue.makeBoolean(isAngleWithinTolerance()));
        networkTable.putValue("balls_shot", NetworkTableValue.makeInteger(ballsShot));
        networkTable.putValue("beambreak_broken", NetworkTableValue.makeBoolean(isBeamBroken()));
        networkTable.putValue("should_aim_at_point", NetworkTableValue.makeBoolean(shouldAimAtShuttlePoints));

        otfSolutionPublisher.set(effectiveTargetLocation);

        // Was Broken, Is broken
        if (wasBeamBreakBroken && !isBeamBroken()) {
            wasBeamBreakBroken = false;
            ballsShot++;
            simulateShotBall();
            ledSubsystem.updateDefaultLED(ballsShot);
            ledSubsystem.setLEDStyle(LedStyle.DEFAULT);
        } else if (isBeamBroken()) {
            wasBeamBreakBroken = true;
            ledSubsystem.setLEDStyle(LedStyle.SHOT);
        }

        ballSimulator.update();
    }

    public void simulateShotBall() {
        Translation3d drivetrainVeloTransform = new Translation3d(drivetrain.getChassisSpeeds().vxMetersPerSecond,
                drivetrain.getChassisSpeeds().vyMetersPerSecond, 0);
        Pose2d drivetrainPose = drivetrain.getPose2d();
        ballSimulator.addBall(new BallState(
                new Pose3d(new Translation3d(drivetrainPose.getX(), drivetrainPose.getY(),
                        0),
                        new Rotation3d(drivetrainPose.getRotation())),
                new Translation3d(
                        shotSolution.launchSpeed() * Math.cos(shotSolution.launchPitchRad()),
                        0,
                        shotSolution.launchSpeed() * Math.sin(shotSolution.launchPitchRad()))
                        .rotateBy(new Rotation3d(drivetrainPose.getRotation()))
                        .plus(drivetrainVeloTransform),
                new Translation3d(0, 100, 0)));
    }

    public boolean getWasBeamBroken() {
        return wasBeamBreakBroken;
    }

    public boolean isBeamBroken() {
        return !beamBreak.get();
    }

    public void solveOTF() {
        ChassisSpeeds speeds = drivetrain.getChassisSpeeds();
        Pose3d goal = FieldConstants.getHubPosition()
                .plus(new Transform3d(0d, 0d, -Units.inchesToMeters(18), new Rotation3d()));

        Pose2d currentPose = drivetrain.getPose2d();

        if (!shouldAimAtShuttlePoints) {
            effectiveTargetLocation = ShootOnTheFlyCalculator.calculateEffectiveTargetLocation(
                    currentPose,
                    goal, speeds, accel,
                    (dist) -> {
                        return ShooterConstants.DISTANCE_TO_SHOOT_VELOCITY.get(dist);
                    },
                    5, 0
            );
        } else {
            effectiveTargetLocation = ShootOnTheFlyCalculator.calculateEffectiveTargetLocation(
                    currentPose,
                    new Pose3d(aimAtPoint.getX(), aimAtPoint.getY(), 0, new Rotation3d()), speeds, accel,
                    (dist) -> {
                        return ShooterConstants.DISTANCE_TO_SHOOT_VELOCITY.get(dist);
                    },
                    5, 0);
        }

        if (currentPose.getX() - goal.getX() < 0 && !shouldAimAtShuttlePoints) {
            effectiveTargetLocation = rotate180(effectiveTargetLocation);
        }

        double targetDistance = currentPose.getTranslation()
                .getDistance(effectiveTargetLocation.toPose2d().getTranslation());
        networkTable.putValue("target_distance", NetworkTableValue.makeDouble(targetDistance));

        shotSolution = BallPhysics.solveBallisticWithIncomingAngle(new Pose3d(currentPose), effectiveTargetLocation,
                Units.degreesToRadians(ShooterConstants.INCOMMING_SHOT_ANGLE));

    }

    public Pose3d getOTFSolution() {
        return effectiveTargetLocation;
    }

    public ShotSolution getShotSolution() {
        return shotSolution;
    }

    public void aimAtShuttlePoint() {
        shouldAimAtShuttlePoints = true;
    }

    public void resetAimPoint() {
        shouldAimAtShuttlePoints = false;
    }

    public Rotation2d getAimAngle() {
        Pose2d targetPose;

        /*
         * This controls whether the robot should be aiming at
         * the closest point on our current side in order to auto shuttle
         * 
         * or aim at the hub to score
         */

        if (shouldAimAtShuttlePoints) {
            Translation2d currentTransform = drivetrain.getPose2d().getTranslation();
            Translation2d aimAtPoint = Translation2d.kZero;
            Translation2d[] aimAtPoints = FieldConstants.getFieldPoints();

            if (aimAtPoints.length > 1) {
                goalAimPosition.set(new Pose3d[] {
                        new Pose3d(aimAtPoints[0].getX(), aimAtPoints[0].getY(), 0, new Rotation3d(0, 0, 0)),
                        new Pose3d(aimAtPoints[1].getX(), aimAtPoints[1].getY(), 0, new Rotation3d(0, 0, 0))
                });

                double distanceA = currentTransform.getSquaredDistance(aimAtPoints[0]);
                double distanceB = currentTransform.getSquaredDistance(aimAtPoints[1]);

                if (distanceA < distanceB) {
                    aimAtPoint = aimAtPoints[0];
                } else {
                    aimAtPoint = aimAtPoints[1];
                }
            }

            this.aimAtPoint = aimAtPoint;
            targetPose = new Pose2d(this.aimAtPoint.getX(), aimAtPoint.getY(), new Rotation2d());
        } else if (isOTFSolution()) {
            targetPose = getOTFSolution().toPose2d();
        } else {
            targetPose = FieldConstants.getHubPosition().toPose2d();
        }

        Pose2d robotPose = drivetrain.getPose2d();
        double offsetX = robotPose.getX() - targetPose.getX(); // Long Side
        double offsetY = robotPose.getY() - targetPose.getY(); // Short Side

        /*
         * Flip field when on blue team or when autonomous
         */

        boolean blueTeamOrAutonomous = (FieldConstants.getAlliance() == Alliance.Blue || DriverStation.isAutonomous());
        double degreeOffset = blueTeamOrAutonomous ? Math.PI : 0;

        return new Rotation2d(MathUtil.angleModulus(Math.atan2(offsetY, offsetX) + degreeOffset));
    }

    public boolean isAngleWithinTolerance() {
        double angleOffset = Math.abs(getAimAngle().getDegrees() - drivetrain.getPose2d().getRotation().getDegrees());
        networkTable.putValue("Angle Offset", NetworkTableValue.makeDouble(angleOffset));

        boolean inTolerance = angleOffset < ShooterConstants.AIM_TOLERANCE
                || Math.abs(180 - angleOffset) < ShooterConstants.AIM_TOLERANCE
                || Math.abs(360 - angleOffset) < ShooterConstants.AIM_TOLERANCE;

        if (inTolerance) {
            ledSubsystem.setLEDStyle(LedStyle.AIMED);
        } else if (ledSubsystem.getLEDStyle() == LedStyle.AIMED) {
            ledSubsystem.setLEDStyle(LedStyle.DEFAULT);
        }

        return inTolerance;
    }

    public boolean isOTFSolution() {
        return !effectiveTargetLocation.equals(new Pose3d());
    }

    public boolean isShotSolution() {
        return !shotSolution.equals(new ShotSolution(0, 0, 0));
    }

    private Pose3d rotate180(Pose3d pose3d) {
        return pose3d.rotateAround(FieldConstants.getHubPosition().getTranslation(), new Rotation3d(0, 0, Math.PI));
    }

}
