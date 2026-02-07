package frc.robot.subsystems;

import java.util.Objects;
import java.util.function.Consumer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem implements Subsystem {
    /*
     * MotorPair<Primary, Follower>
     */
    private MotorPair m_sparkMaxPair;
    /*
     * Hood Spark Is To Adjust Hood Angle
     */
    private SparkMax m_hoodSpark; 
    /*
     * Current Shoot Mode
     */
    private ShootMode m_shootMode = ShootMode.Idle;
    /*
     * Voltage To Set Motor When m_shootMode==ShootMode.Shooting
     */
    private double m_shootRPM = 500;
    /*
     * List Of SpeedConsumers<double, consumer> that will be called once
     * the current RPM goes over the double (RPM) 
     */
    private SparkClosedLoopController shooterClosedLoop = null;
    private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("shooter");
    private double targetHoodAngle = 0;

    public double getTargetHoodAngle() {
        return targetHoodAngle;
    }

    public void setTargetHoodAngle(double targetHoodAngle) {
        this.targetHoodAngle = targetHoodAngle;
    }

    public enum ShootMode {
        Shooting, Idle
    }

    public void addRPM(double addRpm) {
        applyShooterRPM(this.m_shootRPM + addRpm);
    }

    public void applyShooterRPM(double rpm) {
        this.m_shootRPM = MathUtil.clamp(rpm, 0, ShooterConstants.SHOOTER_MAX_RPM);
    }

    public double getShooterRPM() {
        return this.m_shootRPM;
    }

    public void requestShootMode(ShootMode shootMode) {
        this.m_shootMode = shootMode;
        switch (m_shootMode) {
            case Shooting -> {
                shooterClosedLoop.setSetpoint(m_shootRPM, ControlType.kVelocity);
            }

            case Idle -> {
                shooterClosedLoop.setSetpoint(ShooterConstants.SHOOTER_IDLE_RPM, ControlType.kVelocity);
            }
        }
    }

    public ShootMode getShootMode() {
        return m_shootMode;
    }

    public MotorPair getShooterMotors() {
        return m_sparkMaxPair;
    }

    /*
     * Initialize Shooter Motors (Primary + Secondary)
     */

    public ShooterSubsystem setShooterMotors(ShootMotor configPrimary, ShootMotor configFollower) {
        if (!Objects.isNull(m_sparkMaxPair)) {
            System.out.println("[WARNING] - MotorPair Is Already Set");
            return this;
        }

        SparkFlex primary = new SparkFlex(configPrimary.canid(), MotorType.kBrushless);
        SparkFlex follower = new SparkFlex(configFollower.canid(), MotorType.kBrushless);

        SparkFlexConfig primaryConfig = new SparkFlexConfig();
        primaryConfig
                .idleMode(IdleMode.kCoast)
                .inverted(configPrimary.reversed());

        primaryConfig.closedLoop.pid(ShooterConstants.SHOOTER_P, ShooterConstants.SHOOTER_I, ShooterConstants.SHOOTER_D);

        SparkFlexConfig followerConfig = new SparkFlexConfig();
        followerConfig
                .idleMode(IdleMode.kCoast)
                .inverted(configFollower.reversed());

        shooterClosedLoop = primary.getClosedLoopController();

        primary.configure(primaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        followerConfig.follow(configPrimary.canid(), configFollower.reversed());
        m_sparkMaxPair = new MotorPair(primary, follower);

        return this;
    }

    /*
     * Get Hood Motor
     */

    public ShooterSubsystem setHoodMotor(ShootMotor configHood) {
        m_hoodSpark = new SparkMax(configHood.canid(), MotorType.kBrushless);

        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig
            .idleMode(IdleMode.kBrake)
            .inverted(configHood.reversed());

        hoodConfig.encoder
            .positionConversionFactor(ShooterConstants.HOOD_GEAR_RATIO);

        m_hoodSpark.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        return this;
    }

    public double getHoodPosition() {
        return m_hoodSpark.getAbsoluteEncoder().getPosition();
    }

    public SparkMax getHood() {
        return m_hoodSpark;
    }

   /*
    * Periodic Used To Check When getRPM() > target RPM
    */ 

    @Override
    public void periodic() {
        nt.putValue("target_hood_angle", NetworkTableValue.makeDouble(targetHoodAngle));
        nt.putValue("target_shooter_rpm", NetworkTableValue.makeDouble(m_shootRPM));
        nt.putValue("hood_angle", NetworkTableValue.makeDouble(getRPM()));
        nt.putValue("shooter_rpm", NetworkTableValue.makeDouble(getHoodPosition()));

        targetHoodAngle = MathUtil.clamp(targetHoodAngle, ShooterConstants.HOOD_MIN_ANGLE, ShooterConstants.HOOD_MAX_ANGLE);
    }

    /*
     * Gets Current Shooter RPM
     */

    private double getRPM() {
        return Math.abs(m_sparkMaxPair.primary().getAbsoluteEncoder().getVelocity());
    }

    /*
     * Verifies that the motor is set up properly
     */

    public void finish() {
        if (Objects.isNull(m_sparkMaxPair)) {
            System.out.println("[WARNING] - MotorPair Is Null");
            return;
        }
        if (Objects.isNull(m_hoodSpark)) {
            System.out.println("[WARNING] - HoodSpark Is Null");
            return;
        }
        if (Objects.isNull(shooterClosedLoop)) {
            System.out.println("[WARNING] - ShooterClosedLoop Is Null");
            return;
        }
    }

    public record ShootMotor(int canid, boolean reversed) {
        public ShootMotor {
            Objects.requireNonNull(canid);
            Objects.requireNonNull(reversed);
        }
    }

    /*
     * Records (Basically Just Stores Its Inputs, Im going to guess you have never used this Colton)
     * Records just save input(s) (ex. PairName(ClassType Name, ClassType Name2, ...))
     * I usually use them for pairs or tuples, but you can use them with more then how every many
     * Objects.requireNonNull is not required but they are there for saftey
     * You can not set items in the pair, you can only get (basically all inputs are automatically final)
     * To set a pair, make a new pair with the contents of the old pair
     * 
     */

    

    public record MotorPair(SparkFlex primary, SparkFlex follower) {
        public MotorPair {
            Objects.requireNonNull(primary);
            Objects.requireNonNull(follower);
        }
    };

}
