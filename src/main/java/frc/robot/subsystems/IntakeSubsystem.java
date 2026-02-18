package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intake;
    private final SparkMax intakePivot;

    public enum IntakePosition {
        STOWED(120),
        DEPLOYED(0.0);

        private double angle;
        
        private IntakePosition(double angle) {
            this.angle = angle;
        }
    }

    private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("intake");

    private IntakePosition position = IntakePosition.STOWED;
    
    public IntakeSubsystem() {
        intake = new SparkMax(IntakeConstants.INTAKE_ROLLER_PORT, MotorType.kBrushless);
        intakePivot = new SparkMax(IntakeConstants.INTAKE_PIVOT_PORT, MotorType.kBrushless);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.inverted(IntakeConstants.INTAKE_REVERSED);

        intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig intakePivotConfig = new SparkMaxConfig();
        intakeConfig.inverted(IntakeConstants.INTAKE_PIVOT_REVERSED);
        intakePivotConfig.encoder.positionConversionFactor(IntakeConstants.INTAKE_PIVOT_RATIO);

        intakePivot.configure(intakePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {
        nt.putValue("intake_speed", NetworkTableValue.makeDouble(getIntakeSpeed()));
        nt.putValue("intake_set_position", NetworkTableValue.makeString(position.toString()));
        nt.putValue("intake_goal_position", NetworkTableValue.makeDouble(position.angle));
        nt.putValue("intake_position", NetworkTableValue.makeDouble(intakePivot.getEncoder().getPosition()));

        double armTarget = position.angle;
        double ff = IntakeConstants.PIVOT_FEEDFORWARD.calculate(Units.degreesToRadians(armTarget), 0.0);
        double output = IntakeConstants.PIVOT_CONTROLLER.calculate(getIntakePosition(), armTarget);

        intakePivot.set((output + ff) / 12d);

    }

    public void setIntakeSpeed(double speed) {
        intake.set(speed);
    }

    public double getIntakeSpeed() {
        return intake.get();
    }

    public void setIntakePosition(IntakePosition position) {
        this.position = position;
    } 

    public double getIntakePosition() {
        return intakePivot.getEncoder().getPosition();
    }
}
