package mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LEDlight {
    private Servo LED;
    public enum LEDState {WHITE, RED, ORANGE, YELLOW, SAGE, GREEN, AZURE, BLUE, INDIGO, VIOLET}
    public LEDState currentState =  LEDState.WHITE;

    public final double RED = 0.279;
    public final double ORANGE = 0.333;
    public final double YELLOW = 0.35;
    public final double SAGE = 0.444;
    public final double GREEN = 0.500;
    public final double AZURE = 0.555;
    public final double BLUE = 0.6;
    public final double INDIGO = 0.666;
    public final double VIOLET = 0.722;
    public final double WHITE = 1;

    public void init(HardwareMap hm){
        LED = hm.get(Servo.class, "shiny");
    }

    public void Loop(){
        switch (currentState){
            case WHITE:
                LED.setPosition(WHITE);
                break;
            case RED:
                LED.setPosition(RED);
                break;
            case ORANGE:
                LED.setPosition(ORANGE);
                break;
            case YELLOW:
                LED.setPosition(YELLOW);
                break;
            case SAGE:
                LED.setPosition(SAGE);
                break;
            case GREEN:
                LED.setPosition(GREEN);
                break;
            case AZURE:
                LED.setPosition(AZURE);
                break;
            case BLUE:
                LED.setPosition(BLUE);
                break;
            case INDIGO:
                LED.setPosition(INDIGO);
                break;
            case VIOLET:
                LED.setPosition(VIOLET);
                break;
            default:
                LED.setPosition(WHITE);
                break;
        }
    }

    public void setState(LEDState currentState) {
        this.currentState = currentState;
    }

    public LEDState getCurrentState() {
        return currentState;
    }
}
