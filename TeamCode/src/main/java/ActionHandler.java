import android.util.Log;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.EnumMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;

import mechanisms.*;

public class ActionHandler {
    public enum CycleMode {
        SPECIMEN_INTAKING,
        SPECIMEN_SCORING,
        SAMPLE_SCORING,
        CLIMBING
    }
    public enum SpecimenStage {
        EXTENDED,
        INTAKING,
        RETRACTED,
        SPITTING
    }
    public enum SampleStage {
        EXTENDED,
        INTAKING,
        TRANSFERRING,
        LIFTING,
        DEPOSIT,
        IDLE
    }
    private Slides slides;
    private Extendo extendo;
    private Bar bar;
    private Wrist wrist;
    private Intake intake;
    private Claw claw;
    private IntakeWrist intakeWrist;
    private Colorsensor colorSensor;
    private LEDlight light;

    private static boolean intaking, transferring = false;
    private static boolean extendoout = false;
    private static boolean sharing = false;
    private String alliance;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime intakeTimer = new ElapsedTime();
    private boolean waitingForSecondCheck = false;
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();


    public ActionState currentActionState = ActionState.IDLE;
    public CycleMode robotMode = CycleMode.SAMPLE_SCORING;
    public SpecimenStage specimenStage;
    public SampleStage sampleStage;
    enum ActionState {
        IDLE,
        TRANSFER_STAGE_1, //intakewrist in BEFORE, stage1: claw close when wait is done
        TRANSFER_STAGE_2, //flywheel out
        TRANSFER_STAGE_3, //barwrist transfer, flywheel stop
        TRANSFER_STAGE_4, //clawopen
        TRANSFER_STAGE_5,
        CLIP, //delay to wrist move
        WALLPICKUP,
        HIGHBUCKET, //slides up BEFORE
        SLIDESDOWN, //extendo in
        RESETEXTENDO,
        RESETSLIDES,
        NUDGE1, NUDGE2, NUDGE3, CLIPPOS, TRANSFER_STAGE_6, NUDGE4,
        FLIP,
        SPIT, SPIT2, SPIT3,
        THROW1, THROWGRAB
    }

    public void init(Slides s, Extendo e, Bar b, Wrist w, Intake f, Claw c, IntakeWrist iw, Colorsensor cs, LEDlight l, String alliance) {
        slides = s;
        extendo = e;
        bar = b;
        wrist = w;
        intake = f;
        claw = c;
        intakeWrist = iw;
        colorSensor = cs;
        light = l;
        this.alliance = alliance;
        bar.setState(Bar.BarState.WALL);
        claw.setState(Claw.ClawState.CLOSE);
        wrist.setState(Wrist.wristState.WALL);
        extendo.setTargetPos(Extendo.MIN);
        light.setState(LEDlight.LEDState.WHITE);
        transferring = false;
        intaking = false;
        extendoout = false;
    }

    public void Loop(Gamepad gp1, Gamepad gp2) {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gp1);
        currentGamepad2.copy(gp2);
        switch (robotMode) {
            case SAMPLE_SCORING:
                updateSamples();
                if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper && currentActionState == ActionState.IDLE) {
                    nextSampleStage();
                }
                if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper && currentActionState == ActionState.IDLE) {
                    previousSampleStage();
                }
                break;
        }
        if (currentGamepad1.dpad_left) {
            robotMode = CycleMode.SAMPLE_SCORING;
        }
        if (currentGamepad1.dpad_up) {
            robotMode = CycleMode.SPECIMEN_SCORING;
        }
        if (currentGamepad1.dpad_right) {
            robotMode = CycleMode.SPECIMEN_INTAKING;
        }
        TimedActions(); //Update FSM
    }
    public void nextSampleStage() {
        switch (sampleStage) {
            case IDLE:
                sampleStage = SampleStage.EXTENDED;
                break;
            case EXTENDED:
                intake.setState(Intake.intakeState.IN);
                sampleStage = SampleStage.INTAKING;
                break;
            case INTAKING:
                sampleStage = SampleStage.TRANSFERRING;
                currentActionState = ActionState.TRANSFER_STAGE_1;
                timer.reset();
                break;
            case TRANSFERRING:
                sampleStage = SampleStage.LIFTING;
                currentActionState = ActionState.HIGHBUCKET;
                timer.reset();
                break;
            case LIFTING:
                sampleStage = SampleStage.DEPOSIT;
                break;
            case DEPOSIT:
                sampleStage = SampleStage.IDLE;
                currentActionState = ActionState.SLIDESDOWN;
                timer.reset();
                break;
        }
    }
    public void previousSampleStage() {
        switch (sampleStage) {
            case EXTENDED:
                sampleStage = SampleStage.IDLE;
                break;
            case INTAKING:
                intake.setState(Intake.intakeState.STOP);
                sampleStage = SampleStage.EXTENDED;
                break;
            case TRANSFERRING:
                currentActionState = ActionState.TRANSFER_STAGE_1;
                timer.reset();
                sampleStage = SampleStage.EXTENDED
                break;
            case LIFTING:
                sampleStage = SampleStage.TRANSFERRING;
                currentActionState = ActionState.TRANSFER_STAGE_1;
                timer.reset();
                break;
            default:
                //Do nothing
        }
    }
    public void updateSamples() {
        switch (sampleStage) {
            case IDLE:
                extendo.setTargetPos(Extendo.MIN);
                bar.setState(Bar.BarState.NEUTRAL);
                wrist.setState(Wrist.wristState.NEUTRAL);
                claw.setState(Claw.ClawState.CLOSE);
//                slides.setTargetPos(Slides.GROUND); //Handled by Actionhandler FSM
                intakeWrist.setState(IntakeWrist.intakeWristState.IN);
                intake.setState(Intake.intakeState.STOP);
                break;
            case EXTENDED:
                extendo.setTargetPos(Extendo.MAX);
                bar.setState(Bar.BarState.NEUTRAL);
                wrist.setState(Wrist.wristState.NEUTRAL);
                claw.setState(Claw.ClawState.CLOSE);
                slides.setTargetPos(Slides.GROUND);
                intakeWrist.setState(IntakeWrist.intakeWristState.IN);
                intake.setState(Intake.intakeState.STOP);
                break;
            case INTAKING:
                extendo.setTargetPos(Extendo.MAX);
                bar.setState(Bar.BarState.NEUTRAL);
                wrist.setState(Wrist.wristState.NEUTRAL);
                claw.setState(Claw.ClawState.CLOSE);
                slides.setTargetPos(Slides.GROUND);
                intakeWrist.setState(IntakeWrist.intakeWristState.OUT);
                break;
            case TRANSFERRING:
                extendo.setTargetPos(Extendo.MIN);
                //bar, wrist, claw, intakeWrist handled by ActionState FSM
                slides.setTargetPos(Slides.GROUND);
                intake.setState(Intake.intakeState.STOP);
                break;
            case LIFTING:
                extendo.setTargetPos(Extendo.MIN);
//                bar.setState(Bar.BarState.NEUTRAL);
//                wrist.setState(Wrist.wristState.NEUTRAL);
                //bar, wrist handled by ActionState FSM
                claw.setState(Claw.ClawState.CLOSE);
                slides.setTargetPos(Slides.HIGH);
                intakeWrist.setState(IntakeWrist.intakeWristState.IN);
                intake.setState(Intake.intakeState.STOP);
                break;
            case DEPOSIT:
                extendo.setTargetPos(Extendo.MIN);
                bar.setState(Bar.BarState.BUCKET);
                wrist.setState(Wrist.wristState.BUCKET);
                claw.setState(Claw.ClawState.OPEN);
                slides.setTargetPos(Slides.HIGH);
                intakeWrist.setState(IntakeWrist.intakeWristState.IN);
                intake.setState(Intake.intakeState.STOP);
                break;
        }
    }
    public void TimedActions() {
        long elapsedMs = timer.time(TimeUnit.MILLISECONDS);

        switch (currentActionState) {
            //transfer
            case TRANSFER_STAGE_1:
                if (elapsedMs >= 700) {
                    claw.setState(Claw.ClawState.SUPEROPEN);
                    currentActionState = ActionState.TRANSFER_STAGE_2;
                    timer.reset();
                }
                break;
            case TRANSFER_STAGE_2:
                if (elapsedMs >= 400) {
//                if (extendo.getPos()<0) {
                    bar.setState(Bar.BarState.TRANSFER);
                    wrist.setState(Wrist.wristState.TRANSFER);
                    Log.d("hello", "hello!!");
                    currentActionState = ActionState.TRANSFER_STAGE_3;
                    timer.reset();
                }
                break;
            case TRANSFER_STAGE_3:
                if (elapsedMs >= 200) {
                    claw.setState(Claw.ClawState.CLOSE);
                    currentActionState = ActionState.TRANSFER_STAGE_4;
                    timer.reset();
                }
                break;
            case TRANSFER_STAGE_4:
                if (elapsedMs >= 250) {
                    bar.setState(Bar.BarState.NEUTRAL);
                    wrist.setState(Wrist.wristState.NEUTRAL);
                    currentActionState = ActionState.TRANSFER_STAGE_5;
                    timer.reset();
                }
                break;
            case TRANSFER_STAGE_5:
                if (elapsedMs >= 300) {
                    transferring = false;
                    currentActionState = ActionState.IDLE;
                    timer.reset();
                }
                break;
//            case TRANSFER_STAGE_6:
//                if (elapsedMs >= 500) {
//                    intakeWrist.setState(IntakeWrist.intakeWristState.IN);
//                    transferring = false;
//                    currentActionState = ActionState.IDLE;
//                }
//                break;

            //high bucket
            case HIGHBUCKET:
                if (elapsedMs >= 700) {
                    bar.setState(Bar.BarState.BUCKET);
                    wrist.setState(Wrist.wristState.BUCKET);
                    currentActionState = ActionState.IDLE;
                }
                break;
            case SLIDESDOWN:
                if (elapsedMs >= 200){
                    slides.setTargetPos(Slides.GROUND);
                    currentActionState = ActionState.IDLE;
                }
                break;

            //wall pickup
            case WALLPICKUP:
                if (elapsedMs >= 200){
                    bar.setState(Bar.BarState.WALL);
                    currentActionState = ActionState.IDLE;
                }
                break;

            //clipping
            case CLIP:
                if (elapsedMs >= 200) {
                    claw.setState(Claw.ClawState.OPEN);
                    currentActionState = ActionState.IDLE;
                }
                break;


            //flip
            case FLIP:
                if (elapsedMs >= 170){
                    intake.setState(Intake.intakeState.IN);
                    currentActionState = ActionState.IDLE;
                }
                break;

            //nudge sample in intake
            case NUDGE1:
                if (elapsedMs >= 100) {
                    bar.setState(Bar.BarState.TRANSFER);
                    currentActionState = ActionState.NUDGE2;
                    timer.reset();
                }
                break;
            case NUDGE2:
                if (elapsedMs >= 100) {
                    bar.setState(Bar.BarState.NEUTRAL);
                    currentActionState = ActionState.NUDGE3;
                    timer.reset();
                }
                break;
            case NUDGE3:
                if (elapsedMs >= 100) {
                    bar.setState(Bar.BarState.TRANSFER);
                    currentActionState = ActionState.NUDGE4;
                    timer.reset();
                }
                break;
            case NUDGE4:
                if (elapsedMs >= 200){
                    claw.setState(Claw.ClawState.OPEN);
                    bar.setState(Bar.BarState.NEUTRAL);
                    currentActionState = ActionState.IDLE;
                }
                break;
            case CLIPPOS:
                if (elapsedMs >= 500) {
                    bar.setState(Bar.BarState.CLIP);
                    wrist.setState(Wrist.wristState.CLIP);
                    currentActionState = ActionState.IDLE;
                }
                break;
            case SPIT:
                if(elapsedMs>=400){
                    intake.setState(Intake.intakeState.OUT);
                    currentActionState = ActionState.SPIT2;
                    timer.reset();
                }
                break;
            case SPIT2:
                if (elapsedMs>=800) {
                    intake.setState(Intake.intakeState.STOP);
                    intakeWrist.setState(IntakeWrist.intakeWristState.IN);
                    currentActionState=ActionState.IDLE;
                }
                break;
            case THROWGRAB:
                if (elapsedMs>=500) {
                    bar.setState(Bar.BarState.BUCKET);
                    wrist.setState(Wrist.wristState.BUCKET);
                    currentActionState = ActionState.IDLE;
                }
                break;
            case THROW1:
                if (elapsedMs >= 15) {
                    bar.setState(Bar.BarState.CLIP);
                    wrist.setState(Wrist.wristState.CLIP);
                    currentActionState = ActionState.IDLE;
                }
                break;
            default:
                currentActionState = ActionState.IDLE;
                break;
        }
    }

    private void wallPickup() {
        currentActionState = ActionState.WALLPICKUP;
        timer.reset();
    }
    public void clippos() {
        slides.setTargetPos(Slides.MED);
        currentActionState = ActionState.CLIPPOS;
        timer.reset();
    }
    public void clip_down(){
        slides.setTargetPos(Slides.GROUND);
        currentActionState = ActionState.CLIP;
        timer.reset();
    }

    private void intake() {
        intaking = true;
        intake.setState(Intake.intakeState.IN);
        if (extendoout) {
            intakeWrist.setState(IntakeWrist.intakeWristState.OUT);
        } else {
            intakeWrist.setState(IntakeWrist.intakeWristState.SUPEROUT);
        }
    }
    public void throwgrab() {
        slides.setTargetPos(Slides.HIGH);
        currentActionState = ActionState.THROWGRAB;
        timer.reset();
    }
    public void throw2() {
        claw.setState(Claw.ClawState.OPEN);
        currentActionState = ActionState.THROW1;
        timer.reset();
    }
    private void transfer() {
        bar.setState(Bar.BarState.NEUTRAL);
        wrist.setState(Wrist.wristState.TRANSFER);
        claw.setState(Claw.ClawState.CLOSE);
        intakeWrist.setState(IntakeWrist.intakeWristState.TRANSFER);
        currentActionState = ActionState.TRANSFER_STAGE_1;
        timer.reset();
        intake.setState(Intake.intakeState.STOP);
        intaking = false;
    }

    private void nudge(){
        claw.setState(Claw.ClawState.CLOSE);
        bar.setState(Bar.BarState.NEUTRAL);
        wrist.setState(Wrist.wristState.TRANSFER);
        currentActionState = ActionState.NUDGE1;
        timer.reset();
    }

    public void intakeCheck() { //Thanks chatgpt
        if (intaking) {
            // Wait for 300ms before checking again
            if (intakeTimer.milliseconds() >= 600) {
                if (!waitingForSecondCheck) {
                    // First check: Determine if the color is correct
                    boolean correctColor = (alliance.equals("red") && (colorSensor.sensorIsRed() || colorSensor.sensorIsYellow()))
                            || (alliance.equals("blue") && (colorSensor.sensorIsBlue() || colorSensor.sensorIsYellow()));

                    if (correctColor) {
                        // Found the correct color, initiate second check
                        waitingForSecondCheck = true;
                        intakeTimer.reset(); // Reset timer for second check
                    } else {
                        // Handle wrong color immediately
                        boolean wrongColor = (alliance.equals("red") && colorSensor.sensorIsBlue())
                                || (alliance.equals("blue") && colorSensor.sensorIsRed());

                        if (wrongColor) {
                            intake.setState(Intake.intakeState.OUT); // Reverse flywheel to eject
                            intaking = false; // Stop intaking
                        }
                        intakeTimer.reset(); // Reset timer for the next cycle
                    }
                } else {
                    // Second check after 300ms
                    boolean correctColor = (alliance.equals("red") && (colorSensor.sensorIsRed() || colorSensor.sensorIsYellow()))
                            || (alliance.equals("blue") && (colorSensor.sensorIsBlue() || colorSensor.sensorIsYellow()));

                    if (correctColor) {
                        // Confirmed correct color, stop flywheel and intaking
                        intake.setState(Intake.intakeState.STOP);
                        intaking = false;
                    }

                    // Reset state and timer for the next loop
                    waitingForSecondCheck = false;
                    intakeTimer.reset();
                }
            }
        }
    }

    private void highBucket() {
        slides.setTargetPos(Slides.HIGH);
        currentActionState = ActionState.HIGHBUCKET;
        timer.reset();
    }

    private void slidesDown() {
        bar.setState(Bar.BarState.NEUTRAL);
        wrist.setState(Wrist.wristState.TRANSFER);
        currentActionState = ActionState.SLIDESDOWN;
        timer.reset();
    }


    public boolean isIntaking() {
        return intaking;
    }
    public boolean isSharing() {
        return sharing;
    }
    public boolean isTransferring() {return transferring;}
    public boolean isExtendoout() {return extendoout;}
    public boolean isBusy() {
        return currentActionState == ActionState.IDLE;
    }

}