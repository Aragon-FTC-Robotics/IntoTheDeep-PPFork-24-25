import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;

import mechanisms.*;

@TeleOp(name="\uD83D\uDFE5 Red Teleop", group="Linear OpMode")
public class FullTeleOpRed extends LinearOpMode {
    public Bar bar = new Bar();
    public Claw claw = new Claw();
    public Colorsensor colorsensor = new Colorsensor();
    public Slides slides = new Slides();
    public Drivetrain drivetrain = new Drivetrain();
    public Extendo extendo = new Extendo();
    public Intake intake = new Intake();
    public IntakeWrist intakeWrist = new IntakeWrist();
    public Wrist wrist = new Wrist();
    public ActionHandler actionHandler = new ActionHandler();
    public LEDlight led = new LEDlight();
    public ElapsedTime loopTimer = new ElapsedTime();
    public ElapsedTime opTimer = new ElapsedTime();
    public Gamepad gp1;
    public Gamepad gp2;
    private double loopTime, opTime;
    private double[] highestTime;
    @Override
    public void waitForStart() {
        super.waitForStart();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        bar.init(hardwareMap);
        claw.init(hardwareMap);
        colorsensor.init(hardwareMap);
        drivetrain.init(hardwareMap);
        extendo.init(hardwareMap);
        intake.init(hardwareMap);
        intakeWrist.init(hardwareMap);
        slides.init(hardwareMap);
        wrist.init(hardwareMap);
        led.init(hardwareMap);
        actionHandler.init(slides,extendo,bar,wrist,intake,claw,intakeWrist,colorsensor,led, "red");

        gp1 = gamepad1;
        gp2 = gamepad2;

        waitForStart();
        highestTime = new double[]{0.000, 0};
        loopTimer.reset();
        opTimer.reset();
        while(opModeIsActive() && !isStopRequested()) {
            loopTime = loopTimer.milliseconds();
            opTime = opTimer.milliseconds();
            bar.Loop();
            colorsensor.Loop();
            claw.Loop();
            drivetrain.Loop(gp1, gp2); //Gamepad inputs handled by class
            extendo.Loop();
            intake.Loop(gp1, gp2); //Gamepad needed to rumble
            intakeWrist.Loop();
            slides.Loop();
            wrist.Loop();
            led.Loop();
            actionHandler.Loop(gp1, gp2); // :)
            telemetry.addData("High time (ms)", highestTime[0] + "; at " + highestTime[1]);
            telemetry.addData("STATE", actionHandler.currentActionState);
            telemetry.addData("intaking? extendoing? transferring?", actionHandler.isIntaking() + " / " + actionHandler.isExtendoout() + " / " + actionHandler.isTransferring());
            telemetry.addData("slides L/R ", slides.getLPos() + " " + slides.getRPos());
            telemetry.addData("intakewrist state", intakeWrist.getState());
            telemetry.addData("red? / yellow? / blue?", colorsensor.sensorIsRed() + " " + colorsensor.sensorIsYellow() + " " + colorsensor.sensorIsBlue());
            telemetry.update();
            if (loopTime>highestTime[0] || (highestTime[1]-opTime > 5000)) { //If loop time is greater than the highest time OR 5 seconds have passed since last highest time
                highestTime[0] = loopTime; //set highest time to loop time
                highestTime[1] = opTime; //set timestamp to current time
            }
            loopTimer.reset();
        }
    }
}