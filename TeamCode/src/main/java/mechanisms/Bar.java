package mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Bar {
    private Servo barServoRight;
    public enum BarState {TRANSFER, AUTOTRANSFER, WALL, BUCKET, CLIP, AUTOCLIP, NEUTRAL, PARK}
    public BarState currentState = BarState.NEUTRAL;

    public final double TRANSFER = 0.83; //as of jan 25
    public final double AUTOTRANSFER = 0.83;
    public final double WALL = 0.97; //new
    public final double BUCKET = 0.20;
    public final double CLIP = 0.575;
    public final double AUTOCLIP = 0.575;
    public static final double NEUTRAL = 0.6;
    public final double PARK = 0.36;

    public void init(HardwareMap hm) {
        barServoRight = hm.get(Servo.class, "bar");
        barServoRight.setDirection(Servo.Direction.REVERSE);
    }

    public void Loop() {
        switch(currentState) {
            case TRANSFER:
                setPos(TRANSFER);
                break;
            case AUTOTRANSFER:
                setPos(AUTOTRANSFER);
                break;
            case BUCKET:
                setPos(BUCKET);
                break;
            case WALL:
                setPos(WALL);
                break;
            case CLIP:
                setPos(CLIP);
                break;
            case AUTOCLIP:
                setPos(AUTOCLIP);
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
        barServoRight.setPosition(pos);
    }

    public void setState(BarState state) {
        this.currentState = state;
    }

    public String getState(){
        return currentState.name();
    }
}
