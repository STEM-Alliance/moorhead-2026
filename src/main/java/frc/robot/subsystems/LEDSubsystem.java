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

        public double speed;
        public double getSpeed() {
            return this.speed;
        }

        LedStyle(double speed) {
            this.speed = speed;
        }
    }

    public LEDSubsystem(int channel) {
        this.LEDspark = new PWMSparkMax(channel);
    }

    public LedStyle getLEDStyle() {
        return this.LEDStyle;
    }

    public void setLEDStyle(LedStyle style) {
        this.LEDStyle = style;
    }
    public void LEDBalls(int nBalls){
        if (nBalls == 1){
            this.LEDStyle.speed = 0.59;

        }
        else if (nBalls == 10) {

            this.LEDStyle.speed = -0.99;
        }
    }
    @Override
    public void periodic() {
        this.LEDspark.set(LEDStyle.getSpeed());
    }
}
