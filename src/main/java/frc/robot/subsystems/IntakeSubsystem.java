package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intake;
    private final SparkMax intakePivot;
    private boolean waveMidtake = false;

    public enum IntakePosition {
        STOWED(IntakeConstants.PIVOT_MIN),
        DEPLOYED(IntakeConstants.PIVOT_MAX);

        private double angle;

        private IntakePosition(double angle) {
            this.angle = angle;
        }
    }

    private final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("intake");
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
        networkTable.putValue("intake_speed", NetworkTableValue.makeDouble(getIntakeSpeed()));
        networkTable.putValue("intake_set_position", NetworkTableValue.makeString(position.toString()));
        networkTable.putValue("intake_goal_position", NetworkTableValue.makeDouble(position.angle));
        networkTable.putValue("intake_position", NetworkTableValue.makeDouble(intakePivot.getEncoder().getPosition()));
        networkTable.putValue("intake_pivot_output", NetworkTableValue.makeDouble(intakePivot.get()));
        networkTable.putValue("intake_pivot_velocity",
                NetworkTableValue.makeDouble(intakePivot.getEncoder().getVelocity()));

        double armTarget = position.angle;
        // double ff =
        // IntakeConstants.PIVOT_FEEDFORWARD.calculate(Units.degreesToRadians(armTarget),
        // 0.0);
        double intakeShakeWave = Math.sin(System.currentTimeMillis() * IntakeConstants.SHAKE_FREQUENCY)
                * IntakeConstants.SHAKE_AMPLITUDE; // -5 -> 5 | Time * Speed | Period - 1 second
        double outputPosition = Math.min(
                Math.max(
                        IntakeConstants.PIVOT_CONTROLLER.calculate(getIntakePosition(),
                                armTarget + (waveMidtake ? intakeShakeWave : 0)),
                        IntakeConstants.PIVOT_MIN),
                IntakeConstants.PIVOT_MAX);

        intakePivot.set(outputPosition / IntakeConstants.GEAR_RATIO);

    }

    public void setIntakeSpeed(double speed) {
        intake.set(speed);
    }

    public void setShake(boolean shake) {
        this.waveMidtake = shake;
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
