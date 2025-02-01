package mechanisms;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
/*
    In actionhandler, use setTargetPos
    ex. extendo.setTargetPos(Extendo.MAX)
    This allows the position to be changed to numbers outside of the three final ints for fine adjustment etc
 */

public class Extendo {
    private PIDController controller; //from arcrobotics
    private double pid, pidpower;
    public double power;
    public static double p=0.010, i=0, d=0.00015, f=0;
    private DcMotorEx extendo;
//    public enum extendoState {IN, OUT, MED};
    public static final int MAX = 2000;
    public static final int MIN = -250;
    public static final int MED = 900;
    private int targetPos = 0;
    double adjustedP;

    public boolean usingpid = true;

    public void init(HardwareMap hm) {
        controller = new PIDController(p,i,d);
        extendo = hm.get(DcMotorEx.class, "extendo");
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void Loop(double voltage) {
        if (usingpid) {
            adjustedP = p * 13 / voltage;
            controller.setPID(adjustedP,i,d);
            pid = controller.calculate(extendo.getCurrentPosition(), targetPos);
            double ff = f;
            pidpower = pid + ff;
            extendo.setPower(pidpower);
        }
        else {
            extendo.setPower(power);
        }
    }
    public void setTargetPos(int targetPos) {
        this.targetPos = targetPos;
    }
    public double getPos() {
        return extendo.getCurrentPosition();
    }
    public void DANGEROUS_RESET_ENCODERS() {
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setTargetPos(-50);
    }

    public void setPower(double power) {
        usingpid = false;
        this.power = power;
    }
}
