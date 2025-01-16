package mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    private Servo wrist;
    public enum wristState {TRANSFER, BUCKET, WALL, CLIP, NEUTRAL}
    public wristState currentState = wristState.NEUTRAL;

    public final double TRANSFER = 0.150; //as of jan 16 w/ omni intake
    public final double WALL = 0.40;
    public final double BUCKET = 0.90;
    public final double CLIP = 0.31;
    public final double NEUTRAL = 0.00;

    public void init(HardwareMap hm) {
        wrist = hm.get(Servo.class, "wrist");
    }

    public void Loop() {
        switch(currentState) {
            case TRANSFER:
                setPos(TRANSFER);
                break;
            case WALL:
                setPos(WALL);
                break;
            case CLIP:
                setPos(CLIP);
                break;
            case BUCKET:
                setPos(BUCKET);
                break;
            case NEUTRAL:
                setPos(NEUTRAL);
                break;
            default:
                setPos(NEUTRAL);
                break;
        }
    }
    private void setPos(double pos) {
        wrist.setPosition(pos);
    }

    public void setState(wristState state) {
        currentState = state;
    }

    public String getState(){
        return currentState.name();
    }
}
