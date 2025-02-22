package mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo claw;
    public enum ClawState {CLOSE, OPEN, SUPEROPEN}
    public ClawState currentState = ClawState.CLOSE;

    public final double OPEN = 0.55;
    public final double CLOSE = 0.78;
    public final double SUPEROPEN = 0.5;
    public void init(HardwareMap hm) {
        claw = hm.get(Servo.class, "claw");
    }

    public void Loop() {
        switch(currentState) {
            case OPEN:
                setPos(OPEN);
                break;
            case CLOSE:
                setPos(CLOSE);
                break;
            case SUPEROPEN:
                setPos(SUPEROPEN);
            default:
                setPos(OPEN);
                break;
        }
    }


    private void setPos(double pos) {
        claw.setPosition(pos);
    }

    public void setState (ClawState state) {
        currentState = state;
    }

    public String getState(){
        return currentState.name();
    }
}
