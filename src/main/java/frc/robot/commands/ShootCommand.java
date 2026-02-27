// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShootOnTheFlyCalculatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  private final ShooterSubsystem shooter;
  private final IntakeSubsystem intakeSubsystem;
  private final KickerSubsystem kickerSubsystem;
  private final ShootOnTheFlyCalculatorSubsystem shootOnTheFlyCalculatorSubsystem;
  private boolean shotReady = false;

  // intake && hopper

  /** Creates a new ShootCommand. */
  public ShootCommand(ShooterSubsystem shooter, IntakeSubsystem intakeSubsystem, KickerSubsystem kickerSubsystem, ShootOnTheFlyCalculatorSubsystem shootOnTheFlyCalculatorSubsystem) {
    this.shooter = shooter;
    this.intakeSubsystem = intakeSubsystem;
    this.kickerSubsystem = kickerSubsystem;
    this.shootOnTheFlyCalculatorSubsystem = shootOnTheFlyCalculatorSubsystem;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (!shootOnTheFlyCalculatorSubsystem.isOTFSolution() || !shootOnTheFlyCalculatorSubsystem.isShotSolution()) return;

      double goal_rpm = ShooterConstants.SHOT_VELOCITY_TO_RPS.get(shootOnTheFlyCalculatorSubsystem.getShotSolution().launchSpeed()) * 60;
      double goal_pitch = Units.radiansToDegrees(shootOnTheFlyCalculatorSubsystem.getShotSolution().launchPitchRad());

      shooter.setShooterRPM(goal_rpm);
      shooter.setTargetHoodAngle(goal_pitch);

      if (RobotBase.isSimulation()) {
        shotReady = true;
        return;
      }

      if (!shootOnTheFlyCalculatorSubsystem.isAngleWithinTolerance()) return;
      if (!shooter.isReadyToShoot()) return;

      kickerSubsystem.setKickerSpeed(1);
      intakeSubsystem.setIntakeSpeed(1);

      shotReady = true;
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
