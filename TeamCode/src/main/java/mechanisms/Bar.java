package mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Bar {
    private Servo barServoRight;
    public enum BarState {TRANSFER, AUTOTRANSFER, AUTOINIT, WALL, BUCKET, CLIP, AUTOCLIP, NEUTRAL, PARK, DTCLIP1, DTCLIP2, DTWALL, DTWALLSILLY, DTFIRSTCLIP}
    public BarState currentState = BarState.NEUTRAL;
    public double DISPLACEDTICKS = 0.00; //1 tick = 0.02
    public final double TRANSFER = 0.82 + DISPLACEDTICKS; //as of jan 25
    public final double AUTOTRANSFER = 0.83 + DISPLACEDTICKS;
    public final double AUTOINIT = 0.723 + DISPLACEDTICKS;
    public final double WALL = 0.92 + DISPLACEDTICKS; //new
    public final double BUCKET = 0.25 + DISPLACEDTICKS;
    public final double CLIP = 0 + DISPLACEDTICKS;
    public final double AUTOCLIP = 0.585 + DISPLACEDTICKS;
    public final double NEUTRAL = 0.6 + DISPLACEDTICKS;
    public final double DTCLIP1 = 0.13 + DISPLACEDTICKS;
    public final double DTCLIP2 = 0.28 + DISPLACEDTICKS; //og 25
    public final double PARK = 0.41 + DISPLACEDTICKS;
    public final double DTWALL = 0.87 + DISPLACEDTICKS;
    public final double DTWALLSILLY = 0.94 + DISPLACEDTICKS;
    public final double DTFIRSTCLIP = 0.68 + DISPLACEDTICKS;

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
            case AUTOINIT:
                setPos(AUTOINIT);
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
            case DTCLIP1:
                setPos(DTCLIP1);
                break;
            case DTCLIP2:
                setPos(DTCLIP2);
                break;
            case DTWALL:
                setPos(DTWALL);
                break;
            case DTWALLSILLY:
                setPos(DTWALLSILLY);
                break;
            case DTFIRSTCLIP:
                setPos(DTFIRSTCLIP);
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
