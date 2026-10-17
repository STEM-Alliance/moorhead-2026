package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private final PWMSparkMax LEDspark;
    private LedStyle LEDStyle = LedStyle.DEFAULT;

    public enum LedStyle {
        DEFAULT(0.63), // Solid | Orange
        SHOT(0.59), // Solid | Dark Red
        INTAKE(0.69),
        AIMED(0.75);

        private double speed;

        public double getSpeed() {
            return this.speed;
        }

        public void setSpeed(double goal) {
            this.speed = goal;
        }

        LedStyle(double speed) {
            this.speed = speed;
        }
    }

    public LEDSubsystem(int channel) {
        this.LEDspark = new PWMSparkMax(channel);
    }

    public void updateDefaultLED(int ballsShot) {
        LedStyle.DEFAULT.setSpeed(switch (ballsShot) {
            case 1 -> 0.59;
            case 2 -> 0.63;
            case 3 -> 0.65;
            case 4 -> 0.69;
            case 5 -> 0.73;
            case 6 -> 0.79;
            case 7 -> 0.83;
            case 8 -> 0.87;
            case 9 -> 0.91;
            case 10 -> -0.99;
            case 20 -> -0.71;
            case 30 -> -0.65;
            case 40 -> 0.09;
            case 50 -> -0.49;
            case 60 -> -0.39;
            case 70 -> -0.93;
            case 80 -> -0.83;
            case 90 -> -0.59;
            case 100 -> -0.87;
            case 110 -> -0.47;
            case 120 -> -0.21;
            case 130 -> 0.37;
            case 140 -> 0.57;
            case 150 -> -0.89;
            case 160 -> -0.81;
            case 170 -> -0.75;
            case 180 -> -0.17;
            case 190 -> 0.33;
            case 200 -> -0.79;
            case 210 -> 0.15;
            case 211 -> 0.99;
            case 212 -> -0.61;
            default -> LedStyle.DEFAULT.getSpeed();
        });
    }

    public LedStyle getLEDStyle() {
        return LEDStyle;
    }

    public void setLEDStyle(LedStyle style) {
        LEDStyle = style;
    }
   
    @Override
    public void periodic() {
        this.LEDspark.set(LEDStyle.getSpeed());
    }
}
