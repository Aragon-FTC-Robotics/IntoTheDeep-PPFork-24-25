package mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    private Servo wrist;
    public enum wristState {TRANSFER, AUTOTRANSFER, BUCKET, WALL, CLIP, NEUTRAL,PARK}
    public wristState currentState = wristState.NEUTRAL;

    public final double TRANSFER = 0.25; //as of jan 17
    public final double AUTOTRANSFER = 0.25;
    public final double WALL = 0.50;
    public final double BUCKET = 1;
    public final double CLIP = 0.34;
    public final double NEUTRAL = 0.2;
    public final double PARK = 0.84;

    public void init(HardwareMap hm) {
        wrist = hm.get(Servo.class, "wrist");
    }

    public void Loop() {
        switch(currentState) {
            case TRANSFER:
                setPos(TRANSFER);
                break;
            case AUTOTRANSFER:
                setPos(AUTOTRANSFER);
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
            case PARK:
                setPos(PARK);
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
