package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JacksonInject.Value;

import com.fasterxml.jackson.annotation.JacksonInject.Value;

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
    public void LEDBalls(int nBalls){
        switch (nBalls) {
            case 1:
                nBalls = 1;
                this.LEDStyle.speed = 0.59;
                break;
            case 2:
                nBalls = 2;
                this.LEDStyle.speed = 0.63;
                break;
            case 3:
                nBalls = 3;
                this.LEDStyle.speed = 0.65;
                break;
            case 4:
                nBalls = 4;
                this.LEDStyle.speed = 0.69;
                break;
            case 5:
                nBalls = 5;
                this.LEDStyle.speed = 0.73;
                break;
            case 6:
                nBalls = 6;
                this.LEDStyle.speed = 0.79;
                break;
            case 7:
                nBalls = 7;
                this.LEDStyle.speed = 0.83;
                break;
            case 8:
                nBalls = 8;
                this.LEDStyle.speed = 0.87;
                break;
            case 9:
                nBalls = 9;
                this.LEDStyle.speed = 0.91;
                break;
            case 10:
                nBalls = 10;
                this.LEDStyle.speed = -0.99;
                break;
            case 20:
                nBalls = 20;
                this.LEDStyle.speed = -0.71;
                break;
            case 30:
                nBalls = 30;
                this.LEDStyle.speed = -0.65;
                break;
            case 40:
                nBalls = 10;
                this.LEDStyle.speed = 0.09;
                break;
            case 50:
                nBalls = 50;
                this.LEDStyle.speed = -0.49;
                break;
            case 60:
                nBalls = 60;
                this.LEDStyle.speed = -0.39;
                break;
            case 70:
                nBalls = 70;
                this.LEDStyle.speed = -0.93;
                break;
            case 80:
                nBalls = 80;
                this.LEDStyle.speed = -0.83;
                break;
            case 90:
                nBalls = 90;
                this.LEDStyle.speed = -0.59;
                break;
            case 100:
                nBalls = 100;
                this.LEDStyle.speed = -0.87;
                break;
            case 110:
                nBalls = 110;
                this.LEDStyle.speed = -0.47;
                break;
            case 120:
                nBalls = 120;
                this.LEDStyle.speed = -0.21;
                break;
            case 130:
                nBalls = 130;
                this.LEDStyle.speed = 0.37;
                break;
            case 140:
                nBalls = 140;
                this.LEDStyle.speed = 0.57;
                break;
            case 150:
                nBalls = 150;
                this.LEDStyle.speed = -0.89;
                break;
            case 160:
                nBalls = 160;
                this.LEDStyle.speed = -0.81;
                break;
            case 170:
                nBalls = 170;
                this.LEDStyle.speed = -0.75;
                break;
            case 180:
                nBalls = 180;
                this.LEDStyle.speed = -0.17;
                break;
            case 190:
                nBalls = 190;
                this.LEDStyle.speed = 0.33;
                break;
            case 200:
                nBalls = 200;
                this.LEDStyle.speed = -0.79;
                break;
            case 210:
                nBalls = 210;
                this.LEDStyle.speed = 0.15;
                break;
            case 211:
                nBalls = 211;
                this.LEDStyle.speed = 0.99;
                break;
            case 212:
                nBalls = 212;
                this.LEDStyle.speed = -0.61;
                break;
        }
    }
    @Override
    public void periodic() {
        this.LEDspark.set(LEDStyle.getSpeed());
    }
}
