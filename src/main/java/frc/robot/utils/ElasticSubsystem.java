// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

public class ElasticSubsystem extends SubsystemBase {
  /** Creates a new ElasticSubsystem. */
  public double lastTime = 999;

  public ElasticSubsystem() {
    
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
    if (Timer.getMatchTime() < 30 && lastTime > 30) {
      Notification notification = new Notification();
      notification.setTitle("Match Time");
      notification.setLevel(NotificationLevel.ERROR);
      notification.setDescription("30s Remaining");
      Elastic.sendNotification(notification);
    }
    lastTime = Timer.getMatchTime();
  }
}
