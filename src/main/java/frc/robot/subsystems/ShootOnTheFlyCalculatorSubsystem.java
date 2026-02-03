package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.ChassisAccelerations;
import frc.robot.util.ShootOnTheFlyCalculator;
import frc.robot.util.ShootOnTheFlyCalculator.InterceptSolution;

public class ShootOnTheFlyCalculatorSubsystem implements Subsystem {
    private ChassisSpeeds prevSpeeds;
    private long lastExecute = -1;
    private SwerveSubsystem swerveSubsystem;
    private ChassisAccelerations accel = new ChassisAccelerations(0, 0, 0);
    private static InterceptSolution interceptSolution;

    public ShootOnTheFlyCalculatorSubsystem(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        lastExecute = System.currentTimeMillis();
    }

    @Override
    public void periodic() {
        ChassisSpeeds speeds = swerveSubsystem.getChassisSpeeds();
        if (lastExecute != -1) {
            long deltaTime = System.currentTimeMillis() - lastExecute;
            accel = new ChassisAccelerations(speeds, prevSpeeds, deltaTime);
        }

        prevSpeeds = speeds;
        lastExecute = System.currentTimeMillis();

        solveOnTheFly();
    }

    public void solveOnTheFly() {
        ChassisSpeeds speeds = swerveSubsystem.getChassisSpeeds();
        Pose3d goal = FieldConstants.getHubPosition();
        double targetDistance = swerveSubsystem.odometry.getEstimatedPosition().getTranslation()
                .getDistance(goal.toPose2d().getTranslation());
        InterceptSolution solution = ShootOnTheFlyCalculator.solveShootOnTheFly(
                new Pose3d(swerveSubsystem.getPose().getX(), swerveSubsystem.getPose().getY(), 0,
                        new Rotation3d(swerveSubsystem.getRotation2d())),
                goal,
                speeds, accel, FieldConstants.DISTANCE_TO_SHOT_SPEED.get(targetDistance), 5, 0.01

        );
        interceptSolution = solution;
    }

    public static InterceptSolution getInterceptSolution() {
        return interceptSolution;
    }

}
