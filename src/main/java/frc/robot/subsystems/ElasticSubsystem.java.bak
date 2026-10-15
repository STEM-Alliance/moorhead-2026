package frc.robot.subsystems;

import java.awt.Color;
import java.text.DecimalFormat;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElasticSubsystem extends SubsystemBase {
    private final NetworkTable networkTables = NetworkTableInstance.getDefault().getTable("elastic_game");
    private boolean hasPassedAuto = false;
    private final DecimalFormat decimalFormat = new DecimalFormat("#.0");
    private boolean isBlue = false;
    private boolean teamStateUpdated = false;
    private boolean debounce = false;

    @Override
    public void periodic() {
        String gameMessage = DriverStation.getGameSpecificMessage();
        double currentTime = DriverStation.getMatchTime();

        networkTables.putValue("Voltage", NetworkTableValue.makeDouble(RobotController.getBatteryVoltage()));
        
        if (currentTime == -1) {
            if (!hasPassedAuto) {
                networkTables.putValue("Current Action", NetworkTableValue.makeString("Waiting for match to start..."));
            } else {
                networkTables.putValue("Current Action", NetworkTableValue.makeString("Awaiting..."));
            }
        } else if (DriverStation.isAutonomous()) {
            hasPassedAuto = true;
            networkTables.putValue("Current Action", NetworkTableValue.makeString("Auto: "+ decimalFormat.format(DriverStation.getMatchTime()) + "s"));
        } else {
            if (currentTime > 30) {
                if (Math.round((DriverStation.getMatchTime() - 30) % 25) == 0 && !debounce) {
                    debounce = true;
                    isBlue = !isBlue;
                    networkTables.putValue("Active Team", NetworkTableValue.makeString(isBlue ? "#0166b1" : "#ef1c26"));
                } else if (Math.round((DriverStation.getMatchTime() - 30) % 25) != 0) {
                    debounce = false;
                }
                networkTables.putValue("Current Action", NetworkTableValue.makeString("Shift Time Remaining: "+ decimalFormat.format((DriverStation.getMatchTime() - 30) % 25) + "s"));
            } else {
                networkTables.putValue("Current Action", NetworkTableValue.makeString("Endgame Time Remaining: "+ decimalFormat.format(DriverStation.getMatchTime()) + "s"));
            }
        }

        if (gameMessage.length() != 0 && !teamStateUpdated) {
            switch (gameMessage.charAt(0)) {
                case 'R' -> {
                    networkTables.putValue("Active Team", NetworkTableValue.makeString("#0166b1"));
                    isBlue = true;
                    teamStateUpdated = true;
                }
                case 'B' -> {
                    networkTables.putValue("Active Team", NetworkTableValue.makeString("#ef1c26"));
                    isBlue = false;
                    teamStateUpdated = true;
                }
                default -> {
                    networkTables.putValue("Active Team", NetworkTableValue.makeString("#4e4e4e"));
                }
            }
        }
    }

}
