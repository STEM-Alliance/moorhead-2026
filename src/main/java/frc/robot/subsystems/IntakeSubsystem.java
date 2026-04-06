package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intake;
    private final SparkMax intakePivot;
    private boolean wave = false;

    public enum IntakePosition {
        STOWED(IntakeConstants.PIVOT_MIN),
        DEPLOYED(IntakeConstants.PIVOT_MAX);

        private double angle;

        private IntakePosition(double angle) {
            this.angle = angle;
        }
    }

    private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("intake");

    private IntakePosition position = IntakePosition.STOWED;

    public IntakeSubsystem() {
        intake = new TalonFX(IntakeConstants.INTAKE_ROLLER_PORT);
        intakePivot = new SparkMax(IntakeConstants.INTAKE_PIVOT_PORT, MotorType.kBrushless);

        intake.getConfigurator()
                .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(IntakeConstants.INTAKE_REVERSED ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive));

        // intakeConfig.inverted(IntakeConstants.INTAKE_REVERSED);

        // intake.configure(intakeConfig, ResetMode.kResetSafeParameters,
        // PersistMode.kPersistParameters);

        // SparkMaxConfig intakePivotConfig = new SparkMaxConfig();
        // intakeConfig.inverted(IntakeConstants.INTAKE_PIVOT_REVERSED);
        // intakePivotConfig.encoder.positionConversionFactor(IntakeConstants.INTAKE_PIVOT_RATIO);

        // intakePivot.configure(intakePivotConfig, ResetMode.kResetSafeParameters,
        // PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {
        nt.putValue("intake_speed", NetworkTableValue.makeDouble(getIntakeSpeed()));
        nt.putValue("intake_set_position", NetworkTableValue.makeString(position.toString()));
        nt.putValue("intake_goal_position", NetworkTableValue.makeDouble(position.angle));
        nt.putValue("intake_position", NetworkTableValue.makeDouble(intakePivot.getEncoder().getPosition()));
        nt.putValue("intake_pivot_output", NetworkTableValue.makeDouble(intakePivot.get()));
        nt.putValue("intake_pivot_velocity", NetworkTableValue.makeDouble(intakePivot.getEncoder().getVelocity()));

        double armTarget = position.angle;
        // double ff = IntakeConstants.PIVOT_FEEDFORWARD.calculate(Units.degreesToRadians(armTarget), 0.0);
        double sinWave = (Math.sin(System.currentTimeMillis() * 1) - 0.5) * 5; // 0.25-0.75 | Time * Speed
        double output = IntakeConstants.PIVOT_CONTROLLER.calculate(getIntakePosition(), armTarget + (wave ? sinWave : 0));

        intakePivot.set((output) / 12d);

    }

    public void setIntakeSpeed(double speed) {
        intake.set(speed);
    }

    public void setShake(boolean shake) {
        this.wave = shake;
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
