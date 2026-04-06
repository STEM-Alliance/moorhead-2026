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
import frc.robot.Constants.MidtakeConstants;

public class MidtakeSubsystem extends SubsystemBase {
    private final SparkMax hopper;
    
    private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("midtake");

    
    public MidtakeSubsystem() {
        hopper = new SparkMax(MidtakeConstants.MIDTAKE_ROLLERS_PORT, MotorType.kBrushless);

        SparkMaxConfig hopperConfig = new SparkMaxConfig();
        hopperConfig.inverted(MidtakeConstants.MIDTAKE_REVERSED);

        hopper.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {
        nt.putValue("midtake_speed", NetworkTableValue.makeDouble(getMidtakeSpeed()));
    }

    public double getMidtakeSpeed() {
        return hopper.get();
    }

    public void setMidtakeSpeed(double speed) {
        hopper.set(speed);
    }
}
