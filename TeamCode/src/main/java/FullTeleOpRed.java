

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;

import mechanisms.*;

@TeleOp(name="🦐🩸Red Teleop", group="!!!")
public class FullTeleOpRed extends LinearOpMode {
    public Bar bar = new Bar();
    public Claw claw = new Claw();
    public Colorsensor colorsensor = new Colorsensor();
    public Slides slides = new Slides();
    public Drivetrain_robotcentric drivetrain = new Drivetrain_robotcentric();
    public Extendo extendo = new Extendo();
    public Intake intake = new Intake();
    public IntakeWrist intakeWrist = new IntakeWrist();
    public Wrist wrist = new Wrist();
    public ActionHandler actionHandler = new ActionHandler();
    public LEDlight led = new LEDlight();
    public ElapsedTime loopTimer = new ElapsedTime();
    public ElapsedTime opTimer = new ElapsedTime();
    public ElapsedTime lastVoltageCheck = new ElapsedTime();
    public Gamepad gp1;
    public Gamepad gp2;
    public VoltageSensor voltageSensor;
    double currentVoltage = 13;
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
        slides.DANGEROUS_RESET_ENCODERS();
        extendo.DANGEROUS_RESET_ENCODERS();
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        actionHandler.init(slides,extendo,bar,wrist,intake,claw,intakeWrist,colorsensor,led, drivetrain, "red");

        gp1 = gamepad1;
        gp2 = gamepad2;

        waitForStart();
        drivetrain.start();

        highestTime = new double[]{0.000, 0};
        loopTimer.reset();
        opTimer.reset();
        lastVoltageCheck.reset();
        while(opModeIsActive() && !isStopRequested()) {
            loopTime = loopTimer.milliseconds();
            opTime = opTimer.milliseconds();
            bar.Loop();
            colorsensor.Loop();
            claw.Loop();
            drivetrain.Loop(gp1, gp2); //Gamepad inputs handled by class
            if (slides.getLPos() > 2200) {
                drivetrain.slowModeON();
            } else {
                drivetrain.slowModeOFF();
            }
            if (gp1.dpad_up && drivetrain.driveMode == Drivetrain_robotcentric.DriveMode.MANUAL) {
                drivetrain.startSpecScoring();
            }
            extendo.Loop(currentVoltage);
            intake.Loop(gp1, gp2); //Gamepad needed to rumble
            intakeWrist.Loop();
            slides.Loop(currentVoltage);
            wrist.Loop();
            led.Loop();
            actionHandler.Loop(gp1, gp2); /// :)
            if (lastVoltageCheck.milliseconds() > 500) { //check every 500ms
                currentVoltage = voltageSensor.getVoltage();
                lastVoltageCheck.reset();
            }
            telemetry.addData("Loops per second", 1 / loopTimer.seconds());
            telemetry.addData("High time (ms)", highestTime[0] + "; at " + highestTime[1]);
            telemetry.addData("STATE", actionHandler.currentActionState);
            telemetry.addData("intaking? extendoing? transferring? sharing?", actionHandler.isIntaking() + " / " + actionHandler.isExtendoout() + " / " + actionHandler.isTransferring() + " / " + actionHandler.isSharing());
            telemetry.addData("slides L/R ", slides.getLPos() + " " + slides.getRPos());
            telemetry.addData("intakewrist state", intakeWrist.getState());
            telemetry.addData("red? / yellow? / blue?", colorsensor.sensorIsRed() + " " + colorsensor.sensorIsYellow() + " " + colorsensor.sensorIsBlue());
            telemetry.addData("EXTENDO POS | pid?", extendo.getPos() + " " + extendo.usingpid);
            telemetry.addData("voltage / voltage check timer", currentVoltage + " / " + lastVoltageCheck.milliseconds());
            telemetry.update();
            if (loopTime>highestTime[0] || (opTime-highestTime[1] > 5000)) { //If loop time is greater than the highest time OR 5 seconds have passed since last highest time
                highestTime[0] = loopTime; //set highest time to loop time
                highestTime[1] = opTime; //set timestamp to current time
            }
            loopTimer.reset();
        }
    }
}