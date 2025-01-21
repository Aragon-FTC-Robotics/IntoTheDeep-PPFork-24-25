package mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Drivetrain {
    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor rightRear;
    DcMotor leftRear;
    double minSpeed = -1;
    double maxSpeed = 1;
    double MICROSPEED = 0.3;
    public static boolean slowMode = false;
    public void init(HardwareMap hm){
        rightFront = hm.get(DcMotor.class, "rightFront");
        leftFront = hm.get(DcMotor.class, "leftFront");
        rightRear = hm.get(DcMotor.class, "rightRear");
        leftRear = hm.get(DcMotor.class, "leftRear");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void slowModeON(){
        slowMode = true;
    }

    public void slowModeOFF(){
        slowMode = false;
    }



    public void Loop(Gamepad gp1, Gamepad gp2){
//        double y = Math.pow(gp1.left_stick_y, 3); // Remember, Y stick value is reversed
//        double x = Math.pow(gp1.left_stick_x, 3) * -1.1; // Counteract imperfect strafing
//        double rx = Math.pow(gp1.right_stick_x, 3) * -0.6;
        double y = gp1.left_stick_y;
        double x = gp1.left_stick_x * -1.1;
        double rx = gp1.right_stick_x * -1;
        maxSpeed = slowMode ? 0.5 : 1;
        minSpeed = slowMode ? -0.5 : -1;
//        if (gp1.dpad_up) {
//            x = 0; y = -MICROSPEED;
//        }
//        if (gp1.dpad_down) {
//            x = 0; y = MICROSPEED;
//        }
        if (gp1.dpad_right) {
            x = MICROSPEED; y = 0;
        }
        if (gp1.dpad_left) {
            x = -MICROSPEED; y = 0;
        }
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;




        rightFront.setPower(Range.clip(frontRightPower, minSpeed, maxSpeed));
        rightRear.setPower(Range.clip(backRightPower, minSpeed, maxSpeed));
        leftFront.setPower(Range.clip(frontLeftPower, minSpeed, maxSpeed));
        leftRear.setPower(Range.clip(backLeftPower, minSpeed, maxSpeed));
    }
}
