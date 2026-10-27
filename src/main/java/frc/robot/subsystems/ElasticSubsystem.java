package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

import java.text.DecimalFormat;

public class ElasticSubsystem extends SubsystemBase {
    private final NetworkTable networkTables = NetworkTableInstance.getDefault().getTable("elastic_game");
    private boolean completedAuto = false;
    private final DecimalFormat decimalFormat = new DecimalFormat("#0.0");
    private boolean isTeamBlue = false;
    private boolean teamStateUpdated = false;
    private double lastTime = Double.NEGATIVE_INFINITY;

    @Override
    public void periodic() {
        String gameMessage = DriverStation.getGameSpecificMessage();
        double currentTime = DriverStation.getMatchTime();
        double lastUpdateTime = lastTime;

        updateStats(currentTime);

        if (lastUpdateTime <= -1)
            return;

        if (currentTime <= -1) {
            updateAction(completedAuto ? "Waiting for Tele" : "Waiting for match...");
        } else if (DriverStation.isAutonomous()) {
            updateAction("Auto Shift: " + decimalFormat.format(DriverStation.getMatchTime()) + "s");
            completedAuto = true;
        } else if (currentTime > 30) {
            double lastPeriod = (lastUpdateTime - 30) % 25;
            double currentPeriod = (currentTime - 30) % 25;

            if (lastPeriod < currentPeriod && teamStateUpdated) {
                switchTeam(!isTeamBlue ? Alliance.Blue : Alliance.Red);
            }

            updateAction((isTeamBlue ? Alliance.Blue : Alliance.Red).name() + " Shift: " + decimalFormat.format(currentPeriod) + "s");
        } else {
            updateAction("End Shift: " + decimalFormat.format(DriverStation.getMatchTime()) + "s");
            switchTeam(null);
        }

        if (!gameMessage.isEmpty() && !teamStateUpdated) {
            switch (gameMessage.charAt(0)) {
                case 'R' -> switchTeam(Alliance.Red);
                case 'B' -> switchTeam(Alliance.Blue);
                default -> switchTeam(null);
            }
        }
    }

    public void updateAction(String action) {
        networkTables.putValue("Current Action", NetworkTableValue.makeString(action));
    }

    public void switchTeam(Alliance alliance) {
        if (alliance == null) {
            networkTables.putValue("Active Team", NetworkTableValue.makeBoolean(false));
            return;
        }

        System.out.println("Switched to team: " + alliance.name());
        teamStateUpdated = true;
        isTeamBlue = alliance == Alliance.Blue;

        switch (alliance) {
            case Blue -> networkTables.putValue("Active Team", NetworkTableValue.makeBoolean(true));
            case Red -> networkTables.putValue("Active Team", NetworkTableValue.makeBoolean(false));
        }
    }

    public void updateStats(double currentTime) {
        networkTables.putValue("Match Time", NetworkTableValue.makeDouble(currentTime));
        networkTables.putValue("Voltage", NetworkTableValue.makeDouble(RobotController.getBatteryVoltage()));

        if (currentTime < 30 && lastTime > 30) {
            Notification notification = new Notification();
            notification.setTitle("Match Time");
            notification.setLevel(NotificationLevel.ERROR);
            notification.setDescription("30s Remaining");
            Elastic.sendNotification(notification);
        }

        lastTime = currentTime;
    }

}
