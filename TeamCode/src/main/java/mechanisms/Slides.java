package mechanisms;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {
    private PIDController controller;
    public static double p=0.0055, i=0, d=0.0001;
    public static double f=0;
    DcMotorEx slideLeft;
    DcMotorEx slideRight;
    private double pid, pidpower;
    public double power;
    public static final int GROUND = -30;
    public static final int LOW = 200;
    public static final int MED = 1200;
    public static final int HIGH = 2800;
    public int targetPos = 0;
    double adjustedP;
    public boolean usingpid = true;

    public void init(HardwareMap hm) {
        controller = new PIDController(p,i,d);
        slideLeft = hm.get(DcMotorEx.class, "slideLeft");
        slideRight = hm.get(DcMotorEx.class, "slideRight");

        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight.setDirection(DcMotorSimple.Direction.FORWARD);

        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void Loop(double voltage) {
        if (usingpid){
            adjustedP = p * 13/voltage;
            controller.setPID(adjustedP,i,d);
            pid = controller.calculate(slideLeft.getCurrentPosition(), targetPos);
//        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            double ff = 0;
            pidpower = pid + ff;
            slideLeft.setPower(pidpower);
            slideRight.setPower(pidpower);
        }
        else {
            slideLeft.setPower(power);
            slideRight.setPower(power);
        }
    }
    public void setTargetPos(int targetPos) {
        this.targetPos = targetPos;
    }
    public int getLPos(){return slideLeft.getCurrentPosition();}
    public int getRPos(){return slideRight.getCurrentPosition();}
    public void DANGEROUS_RESET_ENCODERS() {
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetPos = GROUND;
    }

    public void setPower(double power){
        usingpid = false;
        this.power = power;
    }
}
