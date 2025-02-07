import android.util.Log;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

import mechanisms.*;

public class ActionHandler {
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

    public ColorState currentColor = ColorState.NOTHING;
    enum ColorState{
        NOTHING, BLUE, RED, YELLOW
    }

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    public ActionState currentActionState = ActionState.IDLE;

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
        if (gp1.share && !sharing) {
            extendo.DANGEROUS_RESET_ENCODERS();
            gp1.rumbleBlips(5);
            sharing = true;
        }
        if (!gp1.share && sharing) {
            sharing = false;
        }
        //clip
        if (gp2.x && !transferring) {
            intakeWrist.setState(IntakeWrist.intakeWristState.IN);
            slides.setTargetPos(Slides.GROUND);
            wrist.setState(Wrist.wristState.WALL);
            wallPickup();
        }
        if (gp2.left_bumper) {
            claw.setState(Claw.ClawState.CLOSE);
        }

        if (gp2.y && !transferring) {
            clippos();
        }
        if (gp2.a && !transferring) {
            clip_down();
        }
        if (gp2.b){
            bar.setState(Bar.BarState.NEUTRAL);
            wrist.setState(Wrist.wristState.NEUTRAL);
        }
        //intake
        if (currentGamepad1.y && !previousGamepad1.y) {
            if (!intaking) {
                intaking = true;
                intake.setState(Intake.intakeState.IN);
                if (extendoout) {
                    intakeWrist.setState(IntakeWrist.intakeWristState.OUT);
                } else {
                    intakeWrist.setState(IntakeWrist.intakeWristState.SUPEROUT);
                }
            } else {
                intaking = false;
                intake.setState(Intake.intakeState.STOP);
                intakeWrist.setState(IntakeWrist.intakeWristState.IN);
            }
            currentColor = ColorState.NOTHING;
        }
        intakeCheck();

        //flip
        if (gp1.a && !transferring){
            intake.setState(Intake.intakeState.OUT);
            currentActionState = ActionState.FLIP;
            timer.reset();
            currentColor = ColorState.NOTHING;
        }

        if (gp1.left_bumper && !transferring) {
            extendo.setTargetPos(Extendo.MIN);
            claw.setState(Claw.ClawState.CLOSE);
            extendoout = false;
            transfer();
            transferring = true;
            currentColor = ColorState.NOTHING;
        }
        if (gp2.left_stick_button && gp2.right_stick_button) {
            gp2.rumbleBlips(2);
        }

        if (gp2.dpad_up && !transferring) {
            highBucket();
        }
        if (gp2.right_bumper){
            claw.setState(Claw.ClawState.OPEN);
        }

        if (gp2.dpad_down && !transferring) {
            slidesDown();
        }

        //extendo
        if (gp1.right_bumper){
            extendo.setTargetPos(Extendo.MAX);
            extendoout = true;
        }
//        if (gp1.right_trigger > 0.8){
//            extendo.setTargetPos(Extendo.MAX);
//            extendoout = true;
//            if (intakeWrist.currentState != IntakeWrist.intakeWristState.IN) {
//                if (isExtendoout()) {
//                    intakeWrist.setState(IntakeWrist.intakeWristState.OUT);
//                } else {
//                    intakeWrist.setState(IntakeWrist.intakeWristState.SUPEROUT);
//                }
//            }
//        }
        if (gp1.left_trigger > 0.8){
            intakeWrist.setState(IntakeWrist.intakeWristState.IN);
            extendo.setTargetPos(Extendo.MIN);
            extendoout = false;
        }
        if (gp1.options) {
            intake.setState(Intake.intakeState.OUT);
            intaking = false;
            currentColor = ColorState.NOTHING;
        }

        //spit
        if (gp1.right_trigger > 0.8 && currentActionState == ActionState.IDLE) {
            intakeWrist.setState(IntakeWrist.intakeWristState.SPIT);
            currentActionState = ActionState.SPIT;
            timer.reset();
            currentColor = ColorState.NOTHING;
        }

        //reset
//        if (gp1.touchpad_finger_1 && gp1.touchpad_finger_2 && gp2.touchpad_finger_1 && gp2.touchpad_finger_2) {
//            resetExtendo();
//            resetSlides();
//            gp1.rumbleBlips(1);
//            gp2.rumbleBlips(1);
//        }

        if (gp1.touchpad_finger_1 && gp1.touchpad_finger_2){
            intakeWrist.setState(IntakeWrist.intakeWristState.IN);
            intake.setState(Intake.intakeState.STOP);
            intaking = false;
            gp1.rumbleBlips(3);
            gp2.rumbleBlips(3);
        }
        light();
        TimedActions();
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

            //reset extendo
            case RESETEXTENDO:
                if (elapsedMs >= 1000) {
                    extendo.DANGEROUS_RESET_ENCODERS();
                    slides.DANGEROUS_RESET_ENCODERS();
                    slides.setPower(0);
                    extendo.setPower(0);
                    extendo.usingpid = true;
                    slides.usingpid = true;
                    currentActionState = ActionState.IDLE;
                }
                break;
            case RESETSLIDES:
                if (elapsedMs >= 1000){
                    slides.setPower(0);
                    slides.usingpid = true;
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
                            currentColor = ColorState.NOTHING;
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

    private void resetExtendoSlides() {
        extendo.setPower(-0.3);
        Log.d("reseting", "extendo");
        slides.setPower(-0.3);
        Log.d("reseting", "slides");
        currentActionState = ActionState.RESETEXTENDO;
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

    public void light(){
        if (colorSensor.sensorIsRed()){
            currentColor = ColorState.RED;
        }
        if (colorSensor.sensorIsBlue()){
            currentColor = ColorState.BLUE;
        }
        if (colorSensor.sensorIsYellow()){
            currentColor = ColorState.YELLOW;
        }

        switch (currentColor){
            case BLUE:
                light.setState(LEDlight.LEDState.BLUE);
                Log.d("LED", "BLUE");
                break;
            case YELLOW:
                light.setState(LEDlight.LEDState.YELLOW);
                Log.d("LED", "YELLOW");
                break;
            case RED:
                light.setState(LEDlight.LEDState.RED);
                Log.d("LED", "RED");
                break;
            case NOTHING:
                light.setState(LEDlight.LEDState.WHITE);
                Log.d("LED", "WHITE");
                break;
            default:
                light.setState(LEDlight.LEDState.WHITE);
                Log.d("LED", "WHITE");
                break;
        }
    }
}