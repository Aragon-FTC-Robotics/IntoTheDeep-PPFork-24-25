package mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LEDshenanigans {
    private Servo LED;
    public enum LEDState {RED, YELLOW, BLUE, WHITE}
    public LEDlight.LEDState currentState =  LEDlight.LEDState.WHITE;

    public final double RED = 0.279;
    public final double YELLOW = 0.35;
    public final double BLUE = 0.6;
    public final double WHITE = 1;

    public double COLOR = 0.279;

    public void init(HardwareMap hm){
        LED = hm.get(Servo.class, "shiny");
    }

    public void Loop(){
        whee();
    }

    public void setState(LEDlight.LEDState currentState) {
        this.currentState = currentState;
    }

    public LEDlight.LEDState getCurrentState() {
        return currentState;
    }

    public void whee(){
        COLOR += 0.001;

        if (COLOR >= 0.722) {
            COLOR = 0.279;
        }

        LED.setPosition(COLOR);
    }

    public void SOS(){

    }
}
