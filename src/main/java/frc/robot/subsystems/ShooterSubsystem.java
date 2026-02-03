package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.Objects;
import java.util.function.Consumer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.SwerveModuleConstants;

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
    private double m_shootVoltage = 0;
    /*
     * List Of SpeedConsumers<double, consumer> that will be called once
     * the current RPM goes over the double (RPM) 
     */
    private LinkedList<SpeedConsumer> m_reqesting = new LinkedList<>();

    public enum ShootMode {
        Shooting, Idle
    }

    public void requestShootMode(ShootMode shootMode) {
        this.m_shootMode = shootMode;
        switch (m_shootMode) {
            case Shooting -> {
                m_sparkMaxPair.primary().setVoltage(m_shootVoltage);
            }

            case Idle -> {
                m_sparkMaxPair.primary().setVoltage(0);
                m_reqesting.clear();
            }
        }
    }

    /*
     * Gets A TargetRPM and a Consumer and accepts Consumer with current
     * Saves SpeedConsumer(targetRPM, m_motorPair) -> LinkedList
     * Awaits until getRPM() > TargetRPM
     * Accepts m_motorPair with current rpm
     */

    public void speedGreaterThen(double targetRPM, Consumer<Double> m_motorPair) {
        if (m_shootMode.equals(ShootMode.Idle)) {
            System.out.println("Shooter Is Idle And Cant Shoot! Cancelled Request. (targetRPM=" + targetRPM + ")");
            return;
        }
        if (getRPM() > targetRPM) {
            System.out.println("RPM Already Higher Then Requested!");
            m_motorPair.accept(getRPM());
            return;
        }
        m_reqesting.add(new SpeedConsumer(targetRPM, m_motorPair));
    }

    public ShootMode getShootMode() {
        return m_shootMode;
    }

    public ShooterSubsystem setShootVoltage(double output_voltage) {
        this.m_shootVoltage = output_voltage;
        return this;
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

        SparkMax primary = new SparkMax(configPrimary.canid(), MotorType.kBrushless);
        SparkMax follower = new SparkMax(configFollower.canid(), MotorType.kBrushless);

        // TODO: Make This Actually Correct

        SparkMaxConfig primaryConfig = new SparkMaxConfig();
        primaryConfig
                .idleMode(IdleMode.kCoast)
                .inverted(configPrimary.reversed());

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig
                .idleMode(IdleMode.kCoast)
                .inverted(configFollower.reversed());

        primary.configure(primaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        followerConfig.follow(configPrimary.canid());
        m_sparkMaxPair = new MotorPair(primary, follower);

        return this;
    }

    /*
     * Get Hood Motor
     */

    public ShooterSubsystem setHoodMotor(ShootMotor configHood) {
        m_hoodSpark = new SparkMax(configHood.canid(), MotorType.kBrushless);

        //TODO: Make This Correct

        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig
            .idleMode(IdleMode.kBrake)
            .inverted(configHood.reversed());

        hoodConfig.encoder
            .positionConversionFactor(SwerveModuleConstants.DRIVE_ROTATION_TO_METER)
            .velocityConversionFactor(SwerveModuleConstants.DRIVE_METERS_PER_MINUTE);

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
        for (SpeedConsumer consumer : m_reqesting) {
            if (getRPM() >= consumer.speed) {
                consumer.consumer.accept(getRPM());
                m_reqesting.remove(consumer);
            }
        }
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

    public record MotorPair(SparkMax primary, SparkMax follower) {
        public MotorPair {
            Objects.requireNonNull(primary);
            Objects.requireNonNull(follower);
        }
    };

    private record SpeedConsumer(double speed, Consumer<Double> consumer) {
        private SpeedConsumer {
            Objects.requireNonNull(speed);
            Objects.requireNonNull(consumer);
        }
    }

}
