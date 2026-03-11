// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms                                                                                                                                                                
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static class ShooterConstants {
    public static final int HOOD_MOTOR_PORT = 10;
    public static final int SHOOTER_LEADER_PORT = 11;
    public static final int SHOOTER_FOLLOWER_PORT = 12;

   public static final boolean SHOOTER_LEADER_INVERTED = false;
    public static final boolean SHOOTER_FOLLOWER_INVERTED = true;
    public static final boolean HOOD_INVERTED = false;
    public static final double SHOOTER_MAX_RPM = 6500.0;

    public static final double HOOD_GEAR_RATIO = (12d / 48d) * (18d / 310d);
    public static final double HOOD_MAX_ANGLE = 32.982;
    public static final double MIN_HOOD_ANGLE = 25.819244; // degrees

    public static final double SHOOTER_P = 0.00;
    public static final double SHOOTER_I = 0.0000001;
    public static final double SHOOTER_D = 0.0000;

    public static final double SHOOTER_kV = 0.0015;
    public static final double SHOOTER_kA = 0;

    public static final PIDController HOOD_PID = new PIDController(0.0165, 0.0, 0.0001);

    public static final InterpolatingDoubleTreeMap DISTANCE_TO_SHOOT_VELOCITY = new InterpolatingDoubleTreeMap();
    static {
      // DISTANCE_TO_SHOT_SPEED.put(2.07, 7.0);
      // // DISTANCE_TO_SHOT_SPEED.put(2.41, 41.0);
      // // DISTANCE_TO_SHOT_SPEED.put(3.20, 45.0);
      // // DISTANCE_TO_SHOT_SPEED.put(3.87, 49.0);
      // // DISTANCE_TO_SHOT_SPEED.put(4.57, 52.0);
      // DISTANCE_TO_SHOT_SPEED.put(4.92, 9.0);

      // DISTANCE_TO_SHOT_SPEED.put(7.583,15.872);
      DISTANCE_TO_SHOOT_VELOCITY.put(Units.feetToMeters(7.41), 2.4);
      DISTANCE_TO_SHOOT_VELOCITY.put(Units.feetToMeters(17.58), 5.19);
      // DISTANCE_TO_SHOT_SPEED.put(22.583,37.5);
    }
    public static final InterpolatingDoubleTreeMap SHOT_VELOCITY_TO_RPS = new InterpolatingDoubleTreeMap();
    static {
            SHOT_VELOCITY_TO_RPS.put(2.177 * 0.5, 22.34);
      SHOT_VELOCITY_TO_RPS.put(2.177, 49.0);
      SHOT_VELOCITY_TO_RPS.put(5.1985, 56.0);
      SHOT_VELOCITY_TO_RPS.put(7.244, 63.0);
    }
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_HOOD_ANGLE = new InterpolatingDoubleTreeMap();
    public static final double INCOMMING_SHOT_ANGLE = -40;
  }

  public static final class IntakeConstants {
    public static final int INTAKE_ROLLER_PORT = 22;
    public static final int INTAKE_PIVOT_PORT = 21;

    public static final double INTAKE_PIVOT_RATIO = (1.0 / 10.0);

    public static final boolean INTAKE_REVERSED = true;
    public static final boolean INTAKE_PIVOT_REVERSED = false;

    public static final ArmFeedforward PIVOT_FEEDFORWARD = new ArmFeedforward(0.41, 6.1, 0.06);

    public static final ProfiledPIDController PIVOT_CONTROLLER = new ProfiledPIDController(0.28, 0.0, 0.01, new Constraints(100, 100));

    public static final double INTAKE_SPEED = 0.4;
    public static final double PIVOT_MIN = 11;
    public static final double PIVOT_MAX = 22.5;
  }

  public static final class MidtakeConstants {
    public static final int MIDTAKE_ROLLERS_PORT = 30;
    public static final boolean MIDTAKE_REVERSED = false;

    public static final double MIDTAKE_SPEED = 0.4;

  }

  public static final class KickerConstants {
    public static final int KICKER_PORT = 40;
    public static final boolean KICKER_REVERSED = false;

    public static final double KICKER_SPEED = 0.25;
  }

  public static class RobotConstants {
    public static final double robotWidthMeters = Units.inchesToMeters(25.0);
    public static final double robotLengthMeters = Units.inchesToMeters(25.0);

    public static final double TOTAL_MASS_KG = 10;
    public static final double MOMENT_OF_INERTIA = 1;
  }

  public static final class FieldConstants {
    public static final double GRAVITY = 9.81;
    public static final double FIELD_LENGTH = PoseConstants.kAprilTagFieldLayout.getFieldLength();
    public static final double FIELD_WIDTH = PoseConstants.kAprilTagFieldLayout.getFieldWidth();
    public static final Translation3d topCenterPointBlue = new Translation3d(
        PoseConstants.kAprilTagFieldLayout.getTagPose(26).get().getX() + Units.inchesToMeters(47) / 2.0,
        PoseConstants.kAprilTagFieldLayout.getFieldWidth() / 2.0,
        Units.inchesToMeters(72.0));

    public static final Translation3d topCenterPointRed = new Translation3d(
        PoseConstants.kAprilTagFieldLayout.getTagPose(9).get().getX() - Units.inchesToMeters(47) / 2.0,
        PoseConstants.kAprilTagFieldLayout.getFieldWidth() / 2.0,
        Units.inchesToMeters(72.0));

    public static Alliance getAlliance() {
      if (DriverStation.getAlliance().isPresent()) {
        return DriverStation.getAlliance().get();
      }

      return Alliance.Blue;
    }

    public static Pose3d getHubPosition() {
      Alliance alliance = getAlliance();
      if (alliance == Alliance.Red) {
        return new Pose3d(topCenterPointRed, new Rotation3d());
      } else {
        return new Pose3d(topCenterPointBlue, new Rotation3d());
      }
    }
  }

  public static class SwerveModuleConstants {
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double COF = 1.2;
  }

  public static class DriveConstants {
    // TODO: Make sure that this is correct - this is from the SDS website but needs
    // empirical verification
    public static final double MAX_MODULE_VELOCITY = 5.21;
    public static final double MAX_ROBOT_VELOCITY = 5.21; // 2.5
    public static final double MAX_ROBOT_RAD_VELOCITY = 12.0; // Approx. Measured rads/sec

    // TODO: ############## REPLACE PLACEHOLDERS ##############
    public static final double MAX_MODULE_CURRENT = 40;

    public static final double TRACK_WIDTH = Units.inchesToMeters(22);
    public static final double WHEEL_BASE = Units.inchesToMeters(22);
    // TODO: Set this for FWERB V2
    public static final Rotation2d GYRO_ANGLE_OFFSET = Rotation2d.fromDegrees(-90);

    public static final PIDController ROTATION_CONTROLLER = new PIDController(5, 0.0, 0.001);

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
        new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0));

    public static final double XY_SPEED_LIMIT = 1.0;
    public static final double Z_SPEED_LIMIT = 1.0;

    public static final double SLOW_MODE_MULT = 0.25;
    public static final double FULL_DRIVE_MODE_MULT = 1.0;
  }

  public static class CommonConstants {
    public static final boolean LOG_INTO_FILE_ENABLED = true;
  }

  public static final class PathPlannerConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5, 0, 0);
    public static final PIDConstants ROTATION_PID = new PIDConstants(0.001, 0, 0);

    public static final PPHolonomicDriveController HOLONOMIC_FOLLOWER_CONTROLLER = new PPHolonomicDriveController(
        TRANSLATION_PID,
        ROTATION_PID);

    public static final RobotConfig ROBOT_CONFIG = new RobotConfig(
        RobotConstants.TOTAL_MASS_KG,
        RobotConstants.MOMENT_OF_INERTIA,
        new ModuleConfig(

            SwerveModuleConstants.WHEEL_DIAMETER / 2,
            DriveConstants.MAX_MODULE_VELOCITY,
            SwerveModuleConstants.COF, // TODO: ############## REPLACE PLACEHOLDERS
                                       // ##############
            DCMotor.getNEO(1),
            DriveConstants.MAX_MODULE_CURRENT, // TODO: ############## REPLACE PLACEHOLDERS ##############
            4),
        DriveConstants.KINEMATICS.getModules());
  }

  public static final class PoseConstants {
    public static final AprilTagFieldLayout kAprilTagFieldLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.kDefaultField);
    public static final double kPositionStdDevX = 0.1;
    public static final double kPositionStdDevY = 0.1;
    public static final double kPositionStdDevTheta = 10;
    public static final double kVisionStdDevY = 5;
    public static final double kVisionStdDevX = 5;
    public static final double kVisionStdDevTheta = 500;
  }
}