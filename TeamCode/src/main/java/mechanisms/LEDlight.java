package mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LEDlight {
    private Servo LED;
    public enum LEDState {RED, YELLOW, BLUE, WHITE}
    public LEDState currentState =  LEDState.WHITE;

    public final double RED = 0.279;
    public final double BLUE = 0.6;
    public final double YELLOW = 0.35;
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
            case BLUE:
                LED.setPosition(BLUE);
                break;
            case YELLOW:
                LED.setPosition(YELLOW);
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
