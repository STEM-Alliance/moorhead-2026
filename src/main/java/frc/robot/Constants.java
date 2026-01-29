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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
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
    public static final int DRIVER_CONTROLLER_PORT = 1;
    public static final int OPERATOR_CONTROLLER_PORT = 2;
  }

  public static class RobotConstants {
    public static final double robotWidthMeters = Units.inchesToMeters(25.0);
    public static final double robotLengthMeters = Units.inchesToMeters(25.0);

    // TODO: ############## REPLACE PLACEHOLDERS ##############
    public static final double TOTAL_MASS_KG = 10;
    public static final double MOMENT_OF_INERTIA = 1;
  }

  public static final class FieldConstants {
    public static final double GRAVITY = 9.81;

    public static final InterpolatingDoubleTreeMap DISTANCE_TO_SHOT_SPEED = new InterpolatingDoubleTreeMap();
        static {
            DISTANCE_TO_SHOT_SPEED.put(2.07, 7.0);
            // DISTANCE_TO_SHOT_SPEED.put(2.41, 41.0);
            // DISTANCE_TO_SHOT_SPEED.put(3.20, 45.0);
            // DISTANCE_TO_SHOT_SPEED.put(3.87, 49.0);
            // DISTANCE_TO_SHOT_SPEED.put(4.57, 52.0);
            DISTANCE_TO_SHOT_SPEED.put(4.92, 9.0);
            // DISTANCE_TO_SHOT_SPEED.put(0.0, 7.0);
            // DISTANCE_TO_SHOT_SPEED.put(5.0, 8.25);
            // DISTANCE_TO_SHOT_SPEED.put(10.0, 10.0);
        }

    public static final Translation3d topCenterPointBlue = new Translation3d(
        PoseConstants.kAprilTagFieldLayout.getTagPose(26).get().getX() + Units.inchesToMeters(47) / 2.0,
        PoseConstants.kAprilTagFieldLayout.getFieldWidth() / 2.0,
        Units.inchesToMeters(72.0));

      public static final Translation3d topCenterPointRed =  new Translation3d(
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
    public static final int PIGEON_ID = 6;

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double STEERING_GEAR_RATIO = 1.d / (150d / 7d);
    public static final double DRIVE_GEAR_RATIO = (1.d / 6.75d);

    public static final double DRIVE_ROTATION_TO_METER = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER;
    public static final double STEER_ROTATION_TO_RADIANS = STEERING_GEAR_RATIO * Math.PI * 2d;
    public static final double DRIVE_METERS_PER_MINUTE = DRIVE_ROTATION_TO_METER / 60d;
    public static final double STEER_RADIANS_PER_MINUTE = STEER_ROTATION_TO_RADIANS / 60d;

    // TODO: ############## REPLACE PLACEHOLDERS ##############
    public static final double WHEEL_FRICTION_COEFFICIENT = 1.2;

    // Actual drive gains
    // public static final double MODULE_KP = 0.5;
    // public static final double MODULE_KD = 0.03;

    // NOTE: This may need additional tuning!
    public static final double MODULE_KP = 0.285;// 0.75628;// 0.7491; //.5;
    public static final double MODULE_KD = 0.0001;// 0.0057682; //0.0076954;

    // --------- Front Left Module --------- \\
    public static final int FL_DRIVE_ID = 3;
    public static final int FL_STEER_ID = 4;
    public static final int FL_ABSOLUTE_ENCODER_PORT = 1;
    public static final double FL_OFFSET_RADIANS = Units.rotationsToRadians(-0.253906) + Math.PI * 0.5 + Math.PI;
    public static final boolean FL_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean FL_MOTOR_REVERSED = true;

    // --------- Front Right Module --------- \\
    public static final int FR_DRIVE_ID = 1;
    public static final int FR_STEER_ID = 2;
    public static final int FR_ABSOLUTE_ENCODER_PORT = 2;
    public static final double FR_OFFSET_RADIANS = Units.rotationsToRadians(-0.413818) + Math.PI * 0.5 + Math.PI;
    public static final boolean FR_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean FR_MOTOR_REVERSED = true;

    // --------- Back Right Module --------- \\
    public static final int BR_DRIVE_ID = 7;
    public static final int BR_STEER_ID = 8;
    public static final int BR_ABSOLUTE_ENCODER_PORT = 3;
    public static final double BR_OFFSET_RADIANS = Units.rotationsToRadians(0.424805) + Math.PI * 0.5 + Math.PI;
    public static final boolean BR_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean BR_MOTOR_REVERSED = true;

    // --------- Back Left Module --------- \\
    public static final int BL_DRIVE_ID = 5;
    public static final int BL_STEER_ID = 6;
    public static final int BL_ABSOLUTE_ENCODER_PORT = 4;
    public static final double BL_OFFSET_RADIANS = Units.rotationsToRadians(0.370117) + Math.PI * 0.5 + Math.PI;
    public static final boolean BL_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean BL_MOTOR_REVERSED = true;

  }

  public static class AimbotConstants {
    public static final PIDController pidController = new PIDController(0.05, 0.0, 0.0012);
  }

  public static class DriveConstants {
    // TODO: Make sure that this is correct - this is from the SDS website but needs
    // empirical verification
    public static final double MAX_MODULE_VELOCITY = 5.21;
    public static final double MAX_ROBOT_VELOCITY = 5.21; // 2.5
    public static final double MAX_ROBOT_RAD_VELOCITY = 12.0; // Approx. Measured rads/sec

    // TODO: ############## REPLACE PLACEHOLDERS ##############
    public static final double MAX_MODULE_CURRENT = 40;

    public static final double TRACK_WIDTH = Units.inchesToMeters(25);
    public static final double WHEEL_BASE = Units.inchesToMeters(25);
    // TODO: Set this for FWERB V2
    public static final Rotation2d NAVX_ANGLE_OFFSET = Rotation2d.fromDegrees(-90);
    // TODO: I'm not going to touch this... but it seems important!
    public static final double DRIVE_BASE_RADIUS = Units.inchesToMeters(15);

    public static final class ModuleIndices {
      public static final int FRONT_LEFT = 1;
      public static final int FRONT_RIGHT = 3;
      public static final int REAR_LEFT = 0;
      public static final int REAR_RIGHT = 2;
    }

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
        new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0));

    public static final double XY_SPEED_LIMIT = 1.0;
    public static final double Z_SPEED_LIMIT = 1.0;
  }

  public static final class CoralConstants {
    public static final double ALIGN_DISTANCE = 13;
    public static final double SENSOR_DISTANCE = 70; // in MM
  }

  public static final class ClimberConstants {
    public static PIDController pidController = new PIDController(0.1, 0, 0);
    public static final int climbMotorPort = 21;
    public static final double motorTop = 20;
    public static final double motorBottom = 0;
  }

  public static final class ElevatorConstants {
    public static final int ELEVATOR_LEADER_PORT = 9;
    public static final int ELEVATOR_FOLLOWER_PORT = 10;
    public static final int ELEVATOR_LIMIT_SWITCH = 0;
    public static final int INTAKE_LIMIT_SWITCH = 16;
    public static final double ELEVATOR_TOP_LIMIT = 120; // change to actual number
    public static final double ELEVATOR_BOTTOM_LIMIT = 0; // change to actual number
    public static final double ELEVATOR_SPEED_LIMIT = 0.25;
    public static final double ELEVATOR_PARK_HEIGHT = 16;
    public static final double ELEVATOR_SPEED_MODIFIER = 0.5;
    public static final double LV1 = 12; // tween this value
    public static final double LV2 = 31.5; // tween this value
    public static final double LV3 = 64; // tween this value
    public static final double LV4 = 119; // tween this value
    public static final double Intake = 7.5;
    public static final int CORAL_LEADER_PORT = 14;
    public static final int CORAL_FOLLOWER_PORT = 15;
    public static final double CORAL_INTAKE_SPEED = 0.2;
    public static final double CORAL_PLACE_SPEED = -0.2;

    // pid valuse
    public static final double kP = 0.01; // need to toon
    public static final double kI = 0; // may not use
    public static final double kD = 0; // may not use
    public static final double PID_TOLERANCE = 0.1;
    // feed forward values
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
  }

  public static class CommonConstants {
    public static final boolean LOG_INTO_FILE_ENABLED = true;
  }

  public static class AlgaeConstants {
    public static final int ALGAE_INTAKE_PORT = 13;
    public static final int ALGAE_MANIP_PORT = 19;
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double ALGAE_HOLD_SPEED = 0.2; // 20 percent
    public static final double ALGAE_INTAKE_SPEED = 0.5; // 50 percent
    public static final double ALGAE_PLACE_SPEED = -0.2;
    public static final int ALGAE_LIMIT_SWITCH = 3;
    public static final double TOP_LIMIT = -50; // check
    public static final double BOTTOM_LIMIT = 0; // check
    public static final double deadband = 0.01;

    public static final double motorOffset = Units.rotationsToRadians(0);
    public static final PIDController PID_CONTROLLER = new PIDController(25, 0.001, 0);
    public static final int encoderPort = 5;
    public static final boolean Reversed = false;
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
            SwerveModuleConstants.WHEEL_FRICTION_COEFFICIENT, // TODO: ############## REPLACE PLACEHOLDERS
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