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
    private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("elastic_game");
    private boolean hasPassedAuto = false;
    private final DecimalFormat decimalFormat = new DecimalFormat("#.0");
    private boolean isBlue = false;
    private boolean hasRecieved = false;
    private boolean debounce = false;

    @Override
    public void periodic() {
        String gameMessage = DriverStation.getGameSpecificMessage();
        double currentTime = DriverStation.getMatchTime();

        nt.putValue("Voltage", NetworkTableValue.makeDouble(RobotController.getBatteryVoltage()));
        
        if (currentTime == -1) {
            if (!hasPassedAuto) {
                nt.putValue("Current Action", NetworkTableValue.makeString("Waiting for match to start..."));
            } else {
                nt.putValue("Current Action", NetworkTableValue.makeString("Awaiting..."));
            }
        } else if (DriverStation.isAutonomous()) {
            hasPassedAuto = true;
            nt.putValue("Current Action", NetworkTableValue.makeString("Auto: "+ decimalFormat.format(DriverStation.getMatchTime()) + "s"));
        } else {
            if (currentTime > 30) {
                if (Math.round((DriverStation.getMatchTime() - 30) % 25) == 0 && !debounce) {
                    debounce = true;
                    isBlue = !isBlue;
                    nt.putValue("Active Team", NetworkTableValue.makeString(isBlue ? "#0166b1" : "#ef1c26"));
                } else if (Math.round((DriverStation.getMatchTime() - 30) % 25) != 0) {
                    debounce = false;
                }
                nt.putValue("Current Action", NetworkTableValue.makeString("Shift Time Remaining: "+ decimalFormat.format((DriverStation.getMatchTime() - 30) % 25) + "s"));
            } else {
                nt.putValue("Current Action", NetworkTableValue.makeString("Endgame Time Remaining: "+ decimalFormat.format(DriverStation.getMatchTime()) + "s"));
            }
        }

        if (gameMessage.length() != 0 && !hasRecieved) {
            switch (gameMessage.charAt(0)) {
                case 'R' -> {
                    nt.putValue("Active Team", NetworkTableValue.makeString("#0166b1"));
                    isBlue = true;
                    hasRecieved = true;
                }
                case 'B' -> {
                    nt.putValue("Active Team", NetworkTableValue.makeString("#ef1c26"));
                    isBlue = false;
                    hasRecieved = true;
                }
                default -> {
                    nt.putValue("Active Team", NetworkTableValue.makeString("#4e4e4e"));
                }
            }
        }
    }

}
