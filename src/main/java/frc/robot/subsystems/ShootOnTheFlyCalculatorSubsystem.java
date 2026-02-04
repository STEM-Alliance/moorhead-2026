package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.ChassisAccelerations;
import frc.robot.util.ShootOnTheFlyCalculator;
import frc.robot.util.ShootOnTheFlyCalculator.InterceptSolution;

public class ShootOnTheFlyCalculatorSubsystem implements Subsystem {
    private ChassisSpeeds prevSpeeds;
    private long lastExecute = -1;
    private ChassisAccelerations accel = new ChassisAccelerations(0, 0, 0);
    private static InterceptSolution interceptSolution;
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;

    public ShootOnTheFlyCalculatorSubsystem(CommandSwerveDrivetrain commandSwerveDrivetrain) {
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        lastExecute = System.currentTimeMillis();
    }

    @Override
    public void periodic() {
        ChassisSpeeds speeds = commandSwerveDrivetrain.getChassisSpeeds();
        if (lastExecute != -1) {
            long deltaTime = System.currentTimeMillis() - lastExecute;
            accel = new ChassisAccelerations(speeds, prevSpeeds, deltaTime);
        }

        prevSpeeds = speeds;
        lastExecute = System.currentTimeMillis();

        solveOnTheFly();
    }

    public void solveOnTheFly() {
        ChassisSpeeds speeds = commandSwerveDrivetrain.getChassisSpeeds();
        Pose3d goal = FieldConstants.getHubPosition();
        Optional<Pose2d> currentPoseOptional = commandSwerveDrivetrain.samplePoseAt(Utils.getCurrentTimeSeconds());
        if (currentPoseOptional.isEmpty()) {
            return;
        };
        Pose2d currentPose = currentPoseOptional.get();
        double targetDistance = currentPose.getTranslation()
                .getDistance(goal.toPose2d().getTranslation());
        InterceptSolution solution = ShootOnTheFlyCalculator.solveShootOnTheFly(
                new Pose3d(currentPose.getX(), currentPose.getY(), 0,
                        new Rotation3d(currentPose.getRotation())),
                goal,
                speeds, accel, FieldConstants.DISTANCE_TO_SHOT_SPEED.get(targetDistance), 5, 0.01

        );
        interceptSolution = solution;
    }

    public static InterceptSolution getInterceptSolution() {
        return interceptSolution;
    }

}
