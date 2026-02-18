package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KickerConstants;

public class KickerSubsystem extends SubsystemBase {
    private final SparkMax kicker;
    
    private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("kicker");

    
    public KickerSubsystem() {
        kicker = new SparkMax(KickerConstants.KICKER_PORT, MotorType.kBrushless);

        SparkMaxConfig kickerConfig = new SparkMaxConfig();
        kickerConfig.inverted(KickerConstants.KICKER_REVERSED);

        kicker.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {
        nt.putValue("kicker_speed", NetworkTableValue.makeDouble(getKickerSpeed()));
    }

    public double getKickerSpeed() {
        return kicker.get();
    }

    public void setKickerSpeed(double speed) {
        kicker.set(speed);
    }
}
