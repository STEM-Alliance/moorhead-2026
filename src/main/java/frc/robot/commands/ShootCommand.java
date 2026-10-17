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
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShotRegressionCoefficients;
import frc.robot.subsystems.ShootOnTheFlyCalculatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  private final ShooterSubsystem shooter;

  private final ShootOnTheFlyCalculatorSubsystem shootOnTheFlyCalculatorSubsystem;
  private boolean shotReady = false;
  private final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("ShootVelocity");

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
    double launchSpeed = shootOnTheFlyCalculatorSubsystem.getShotSolution().launchSpeed();
    networkTable.putValue("Linear", NetworkTableValue.makeDouble(ShotRegressionCoefficients.linear(launchSpeed)));
    networkTable.putValue("Exp", NetworkTableValue.makeDouble(ShotRegressionCoefficients.exponential(launchSpeed)));
    networkTable.putValue("Cubic", NetworkTableValue.makeDouble(ShotRegressionCoefficients.cubic(launchSpeed)));
    networkTable.putValue("Eyeball", NetworkTableValue.makeDouble(ShooterConstants.SHOT_VELOCITY_TO_RPS.get(launchSpeed) * 45));

    double goalRPM = ShotRegressionCoefficients.cubic(launchSpeed);
    double goalPitch = 90
        - Units.radiansToDegrees(shootOnTheFlyCalculatorSubsystem.getShotSolution().launchPitchRad());

    shooter.setShooterRPM(goalRPM);
    shooter.setTargetHoodAngle(goalPitch);

    networkTable.putValue("IsAngleWithinTolerance",
        NetworkTableValue.makeBoolean(shootOnTheFlyCalculatorSubsystem.isAngleWithinTolerance()));
    networkTable.putValue("IsReadyToShoot",
        NetworkTableValue.makeBoolean(shootOnTheFlyCalculatorSubsystem.isAngleWithinTolerance()));

    shotReady = getIsShotReady();
  }

  private boolean getIsShotReady() {
    if (RobotBase.isSimulation() && shootOnTheFlyCalculatorSubsystem.isAngleWithinTolerance()) {
      return true;
    } else if (shootOnTheFlyCalculatorSubsystem.isAngleWithinTolerance()
        && shooter.isReadyToShoot()) {
      return true;
    }
    return false;
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
