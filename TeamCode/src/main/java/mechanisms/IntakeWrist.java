package mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeWrist {
    private Servo intakeWrist;
    public enum intakeWristState {IN, OUT, SUPEROUT, TRANSFER, SPIT, SUPERALMOSTOUT};
    public intakeWristState currentState = intakeWristState.IN;
    public final double IN = 0.16;
    public final double OUT = 0.95; //Extendo out
    public final double SUPEROUT = 0.95; //Extendo in
    public final double SUPERALMOSTOUT = 0.90;
    public final double TRANSFER = 0.35;
    public final double SPIT = 0.8;
    public void init(HardwareMap hm) {
        intakeWrist = hm.get(Servo.class, "intakeWrist");
    }
    public void Loop() {
        switch (currentState){
            case IN:
                setPosition(IN);
                break;
            case OUT:
                setPosition(OUT);
                break;
            case SUPEROUT:
                setPosition(SUPEROUT);
                break;
            case TRANSFER:
                setPosition(TRANSFER);
                break;
            case SPIT:
                setPosition(SPIT);
                break;
            case SUPERALMOSTOUT:
                setPosition(SUPERALMOSTOUT);
                break;
        }
    }
    private void setPosition(double pos) {
        intakeWrist.setPosition(pos);
    }
    public void setState(intakeWristState state) {
        currentState = state;
    }
    public String getState() {
        return currentState.name();
    }
}
