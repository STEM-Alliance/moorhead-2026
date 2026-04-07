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
        else if (nBalls == 2) {

            this.LEDStyle.speed = 0.63;
        }
        else if (nBalls == 3) {

            this.LEDStyle.speed = 0.65;
        }
        else if (nBalls == 4) {

            this.LEDStyle.speed = 0.69;
        }
        else if (nBalls == 5) {

            this.LEDStyle.speed = 0.73;
        }
        else if (nBalls == 6) {

            this.LEDStyle.speed = 0.79;
        }
        else if (nBalls == 7) {

            this.LEDStyle.speed = 0.83;
        }
        else if (nBalls == 8) {

            this.LEDStyle.speed = 0.87;
        }
        else if (nBalls == 9) {

            this.LEDStyle.speed = 0.91;
        }
        else if (nBalls == 10) {

            this.LEDStyle.speed = -0.99;
        }
        else if (nBalls == 20) {

            this.LEDStyle.speed = -0.71;
        }
        else if (nBalls == 30) {

            this.LEDStyle.speed = -0.65;
        }
        else if (nBalls == 40) {

            this.LEDStyle.speed = 0.09;
        }
        else if (nBalls == 50) {

            this.LEDStyle.speed = -0.49;
        }
        else if (nBalls == 60) {

            this.LEDStyle.speed = -0.39;
        }
        else if (nBalls == 70) {

            this.LEDStyle.speed = -0.93;
        }
        else if (nBalls == 80) {

            this.LEDStyle.speed = -0.83;
        }
        else if (nBalls == 90) {

            this.LEDStyle.speed = -0.59;
        }
        else if (nBalls == 100) {

            this.LEDStyle.speed = -0.87;
        }
        else if (nBalls == 110) {

            this.LEDStyle.speed = -0.47;
        }
        else if (nBalls == 120) {

            this.LEDStyle.speed = -0.21;
        }
        else if (nBalls == 130) {

            this.LEDStyle.speed = 0.37;
        }
        else if (nBalls == 140) {

            this.LEDStyle.speed = 0.57;
        }
        else if (nBalls == 150) {

            this.LEDStyle.speed = -0.89;
        }
        else if (nBalls == 160) {

            this.LEDStyle.speed = -0.81;
        }
        else if (nBalls == 170) {

            this.LEDStyle.speed = -0.75;
        }
        else if (nBalls == 180) {

            this.LEDStyle.speed = -0.17;
        }
        else if (nBalls == 190) {

            this.LEDStyle.speed = 0.33;
        }
        else if (nBalls == 200) {

            this.LEDStyle.speed = -0.79;
        }
        else if (nBalls == 210) {

            this.LEDStyle.speed = 0.15;
        }
        else if (nBalls == 211) {

            this.LEDStyle.speed = 0.99;
        }
        else if (nBalls == 212) {

            this.LEDStyle.speed = -0.61;
        }
        
    }
    @Override
    public void periodic() {
        this.LEDspark.set(LEDStyle.getSpeed());
    }
}
