// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.KickerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShotRegressionCoefficients;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.MidtakeSubsystem;
import frc.robot.subsystems.ShootOnTheFlyCalculatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  private final ShooterSubsystem shooter;

  private final ShootOnTheFlyCalculatorSubsystem shootOnTheFlyCalculatorSubsystem;
  private boolean shotReady = false;
  private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("ShootVelocity");


  // intake && hopper

  /** Creates a new ShootCommand. */
  public ShootCommand(ShooterSubsystem shooter, ShootOnTheFlyCalculatorSubsystem shootOnTheFlyCalculatorSubsystem) {
    this.shooter = shooter;
    this.shootOnTheFlyCalculatorSubsystem = shootOnTheFlyCalculatorSubsystem;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shotReady = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!shootOnTheFlyCalculatorSubsystem.isOTFSolution() || !shootOnTheFlyCalculatorSubsystem.isShotSolution())
      return;
    double launch_speed = shootOnTheFlyCalculatorSubsystem.getShotSolution().launchSpeed();
    nt.putValue("Linear", NetworkTableValue.makeDouble(ShotRegressionCoefficients.linear(launch_speed)));
    nt.putValue("Exp", NetworkTableValue.makeDouble(ShotRegressionCoefficients.exponential(launch_speed)));
    nt.putValue("Cubic", NetworkTableValue.makeDouble(ShotRegressionCoefficients.cubic(launch_speed)));
    nt.putValue("Old Exp", NetworkTableValue.makeDouble(getRPM(launch_speed) * 1000d));
        nt.putValue("Eyeball", NetworkTableValue.makeDouble(ShooterConstants.SHOT_VELOCITY_TO_RPS.get(launch_speed) * 45));


    double goal_rpm = ShotRegressionCoefficients.cubic(launch_speed);
    double goal_pitch = 90
        - Units.radiansToDegrees(shootOnTheFlyCalculatorSubsystem.getShotSolution().launchPitchRad());

    shooter.setShooterRPM(goal_rpm);
    shooter.setTargetHoodAngle(goal_pitch);

    nt.putValue("IsAngleWithinTolerance", NetworkTableValue.makeBoolean(shootOnTheFlyCalculatorSubsystem.isAngleWithinTolerance()));
    nt.putValue("IsReadyToShoot", NetworkTableValue.makeBoolean(shootOnTheFlyCalculatorSubsystem.isAngleWithinTolerance()));
    if (RobotBase.isSimulation() && shootOnTheFlyCalculatorSubsystem.isAngleWithinTolerance()) {
        shotReady = true;
    } else if (shootOnTheFlyCalculatorSubsystem.isAngleWithinTolerance()
        && shooter.isReadyToShoot()) {
      shotReady = true;
    }
    ;

  }

  public double getRPM(double shotVelocity) {
    return -1.63897 + 2.26834 * Math.log(shotVelocity);
    // return (0.240186 * shotVelocity)+0.965213;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shotReady;
  }
}
