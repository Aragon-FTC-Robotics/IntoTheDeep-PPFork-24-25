import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import mechanisms.Bar;
import mechanisms.Claw;
import mechanisms.Colorsensor;
import mechanisms.Drivetrain;
import mechanisms.Extendo;
import mechanisms.Intake;
import mechanisms.IntakeWrist;
import mechanisms.Slides;
import mechanisms.Wrist;
@Disabled
@TeleOp(name="field centirc..... hello..........", group="Linear OpMode")
public class FieldCentricDt extends LinearOpMode {
    public Drivetrain drivetrain = new Drivetrain();
    public ElapsedTime loopTimer = new ElapsedTime();
    public ElapsedTime opTimer = new ElapsedTime();
    public Gamepad gp1;
    public Gamepad gp2;
    private double loopTime, opTime;
    private double[] highestTime = new double[2];
    @Override
    public void waitForStart() {
        super.waitForStart();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);
        gp1 = gamepad1;
        gp2 = gamepad2;

        waitForStart();
        highestTime = new double[]{0.000, 0};
        loopTimer.reset();
        opTimer.reset();
        while(opModeIsActive() && !isStopRequested()) {
            loopTime = loopTimer.milliseconds();
            opTime = opTimer.milliseconds();
            drivetrain.Loop(gp1, gp2); //Gamepad inputs handled by class
            telemetry.addData("Loop time (ms)", loopTime);
            telemetry.addData("Op time (ms)", opTime);
            telemetry.addData("High time (ms)", highestTime[0] + "; at " + highestTime[1]);
            telemetry.update();
            if (loopTime>highestTime[0] || (highestTime[1]-opTime > 5000)) { //If loop time is greater than the highest time OR 5 seconds have passed since last highest time
                highestTime[0] = loopTime; //set highest time to loop time
                highestTime[1] = opTime; //set timestamp to current time
            }
            loopTimer.reset();
        }
    }
}