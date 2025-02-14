import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.*;
import com.pedropathing.util.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import mechanisms.Bar;
import mechanisms.Claw;
import mechanisms.Extendo;
import mechanisms.Intake;
import mechanisms.IntakeWrist;
import mechanisms.Slides;
import mechanisms.Wrist;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Regional auo", group = "Auto")
public class Auto_Regionals extends OpMode {
    private Bar bar;
    private Claw claw;
    private Extendo extendo;
    private Intake intake;
    private IntakeWrist intakeWrist;
    private Slides slides;
    private Wrist wrist;
    private Servo led;
    private Follower follower;
    private Timer pathTime, totalTime, loopTime;
    private int pathState = 0;


    private static final Pose STARTPOSE = new Pose(6.75, 60, Math.toRadians(180));
    private static final Pose PRELOADPOSE = new Pose(45, 67, Math.toRadians(180));
    private static final Pose PREPARE1POSE = new Pose(57,34,Math.toRadians(180));
    private static final Pose PREPARE1CONTROL = new Pose(19, 69);
    private static final Pose PREPARE1CONTROL2 = new Pose(24, 28);
    private static final Pose PUSH1POSE = new Pose(20,23,Math.toRadians(180));
    private static final Pose PUSH1CONTROL = new Pose(65,16);
    private static final Pose PREPARE2POSE = new Pose(54, 14.7, Math.toRadians(180));;
    private static final Pose PREPARE2CONTROL = new Pose(88,15);
    private static final Pose PUSH2POSE = new Pose(20, 15);
    private static final Pose PREPARE3POSE = new Pose(54, 6, Math.toRadians(180));
    private static final Pose PREPARE3CONTROL = new Pose(67, 16);
    private static final Pose PUSH3POSE = new Pose(18, 7, Math.toRadians(180));
    private static final Pose SCORE1MID = new Pose(12, 33, Math.toRadians(240));
    //Push3 -> score1 mid -> score1pose (with score1control)
    private static final Pose SCORE1CONTROL = new Pose(22, 77);
    private static final Pose SCORE1POSE = new Pose(39, 72.6, Math.toRadians(180));
    private static final Pose WALLPOSE = new Pose(8.3, 24, Math.toRadians(180));
    private static final Pose SCORECONTROL = new Pose(19, 36);
    private static final Pose SCORECONTROL2 = new Pose(14, 94);
    private static final Pose SCORETOWALLCONTROL = new Pose(23, 79);
    private static final Pose SCORETOWALLCONTROL2 = new Pose(40, 26);
    private static final Pose SCORE2POSE = new Pose(43.1, 72.6+2, Math.toRadians(180));
    private static final Pose SCORE3POSE =new Pose(43.1, 72.6+4, Math.toRadians(180));
    private static final Pose SCORE4POSE = new Pose(43.1, 72.6+6, Math.toRadians(180));
    private static final Pose PARK = new Pose(16, 21, Math.toRadians(-105));

    private PathChain scorePreload, pushSamples, score1, score1ToWall, score2, score2ToWall, score3, score3ToWall, score4, park;

    private void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(STARTPOSE), new Point(PRELOADPOSE))))
                .setLinearHeadingInterpolation(STARTPOSE.getHeading(), PRELOADPOSE.getHeading())
                .build();
        pushSamples = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(PRELOADPOSE), new Point(PREPARE1CONTROL), new Point(PREPARE1CONTROL2), new Point(PREPARE1POSE))))
                .setConstantHeadingInterpolation(PRELOADPOSE.getHeading())
                .addPath(new Path(new BezierCurve(new Point(PREPARE1POSE), new Point(PUSH1CONTROL), new Point(PUSH1POSE))))
                .setConstantHeadingInterpolation(PRELOADPOSE.getHeading())
                .addPath(new Path(new BezierCurve(new Point(PUSH1POSE), new Point(PREPARE2CONTROL), new Point(PREPARE2POSE))))
                .setConstantHeadingInterpolation(PRELOADPOSE.getHeading())
                .addPath(new Path(new BezierLine(new Point(PREPARE2POSE), new Point(PUSH2POSE))))
                .setConstantHeadingInterpolation(PRELOADPOSE.getHeading())
                .addPath(new Path(new BezierCurve(new Point(PUSH2POSE), new Point(PREPARE3CONTROL), new Point(PREPARE3POSE))))
                .setConstantHeadingInterpolation(PRELOADPOSE.getHeading())
                .addPath(new Path(new BezierLine(new Point(PREPARE3POSE), new Point(PUSH3POSE))))
                .setConstantHeadingInterpolation(PRELOADPOSE.getHeading())
                .build();
        score1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(PUSH3POSE), new Point(SCORE1MID))))
                .setLinearHeadingInterpolation(PUSH3POSE.getHeading(), SCORE1MID.getHeading())
                .addPath(new Path(new BezierCurve(new Point(SCORE1MID), new Point(SCORE1CONTROL), new Point(SCORE1POSE))))
                .setLinearHeadingInterpolation(SCORE1MID.getHeading(), SCORE1POSE.getHeading())
                .build();
        score1ToWall = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(SCORE1POSE), new Point(SCORETOWALLCONTROL), new Point(SCORETOWALLCONTROL2), new Point(WALLPOSE))))
                .setLinearHeadingInterpolation(SCORE1POSE.getHeading(), WALLPOSE.getHeading())
                .build();
        score2 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(WALLPOSE), new Point(SCORECONTROL), new Point(SCORECONTROL2), new Point(SCORE2POSE))))
                .setLinearHeadingInterpolation(WALLPOSE.getHeading(), SCORE2POSE.getHeading())
                .build();
        score2ToWall = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(SCORE2POSE), new Point(SCORETOWALLCONTROL), new Point(SCORETOWALLCONTROL2), new Point(WALLPOSE))))
                .setLinearHeadingInterpolation(SCORE2POSE.getHeading(), WALLPOSE.getHeading())
                .build();
        score3 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(WALLPOSE), new Point(SCORECONTROL), new Point(SCORECONTROL2), new Point(SCORE3POSE))))
                .setLinearHeadingInterpolation(WALLPOSE.getHeading(), SCORE3POSE.getHeading())
                .build();
        score3ToWall = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(SCORE3POSE), new Point(SCORETOWALLCONTROL), new Point(SCORETOWALLCONTROL2), new Point(WALLPOSE))))
                .setLinearHeadingInterpolation(SCORE3POSE.getHeading(), WALLPOSE.getHeading())
                .build();
        score4 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(WALLPOSE), new Point(SCORECONTROL), new Point(SCORECONTROL2), new Point(SCORE4POSE))))
                .setLinearHeadingInterpolation(WALLPOSE.getHeading(), SCORE3POSE.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(SCORE4POSE), new Point(PARK))))
                .setLinearHeadingInterpolation(SCORE4POSE.getHeading(), PARK.getHeading())
                .build();
    }
    private void updatePaths() {
        switch (pathState) {
            case 0: //clip #1
                extendo.setTargetPos(-100);
                intakeWrist.setState(IntakeWrist.intakeWristState.IN);
                bar.setState(Bar.BarState.DTCLIP1);
                wrist.setState(Wrist.wristState.DTCLIP);
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1: //leaving
                if (Math.abs(follower.getPose().getX() - PRELOADPOSE.getX())<10) {
                    bar.setState(Bar.BarState.DTCLIP2);
                    follower.followPath(pushSamples); ///fix after michael changes it to a chain
                    setPathState(2);
                }
                break;
            case 2: //open claw
                if (pathTime.getElapsedTimeSeconds() > 0.4) { /// find exact timing
                    claw.setState(Claw.ClawState.OPEN);
                    setPathState(3);
                }
                break;
            case 3: // go to push samples
                if(follower.getCurrentTValue() > 0.2) {
                    bar.setState(Bar.BarState.WALL);
                    wrist.setState(Wrist.wristState.WALL);
                    setPathState(4);
                }
                break;
            case 4: // grab score 1 (technically 2)
                if (!follower.isBusy()) {
                    claw.setState(Claw.ClawState.CLOSE);
                    setPathState(5);
                }
                break;
            case 5: // get into position/move to bar/clip
                if (pathTime.getElapsedTimeSeconds() > 0.35) {
                    bar.setState(Bar.BarState.CLIP);
                    wrist.setState(Wrist.wristState.CLIP);
                    slides.setTargetPos(Slides.MED);
                    follower.setMaxPower(0.85);
                    follower.followPath(score1);
                    setPathState(6);
                }
                break;
            case 6: //let go of clip after clipped
                if (Math.abs(follower.getPose().getX() - SCORE1POSE.getX()) < 5){
                    claw.setState(Claw.ClawState.OPEN);
                    follower.followPath(score1ToWall);
                    setPathState(7);
                }
                break;
            case 7: // get ready to grab 2nd clip
                if (follower.getCurrentTValue() > 0.25) {
                    bar.setState(Bar.BarState.WALL);
                    wrist.setState(Wrist.wristState.WALL);
                    slides.setTargetPos(Slides.GROUND);
                    setPathState(8);
                }
                break;
            case 8: // grab 2nd clip
                if (!follower.isBusy()) { /// make sure it works
                    claw.setState(Claw.ClawState.CLOSE);
                    setPathState(9);
                }
                break;
            case 9: // go to clip 2nd clip
                if (pathTime.getElapsedTimeSeconds() > 0.35) {
                    bar.setState(Bar.BarState.CLIP);
                    wrist.setState(Wrist.wristState.CLIP);
                    slides.setTargetPos(Slides.MED);
                    follower.setMaxPower(1);
                    follower.followPath(score2);
                    setPathState(10);
                }
                break;
            case 10: // let go of clip
                if (Math.abs(follower.getPose().getX() - SCORE2POSE.getX()) < 5){
                    claw.setState(Claw.ClawState.OPEN);
                    follower.followPath(score2ToWall);
                    setPathState(11);
                }
                break;
            case 11: // get ready for clip #3
                if (follower.getCurrentTValue() > 0.25) {
                    bar.setState(Bar.BarState.WALL);
                    wrist.setState(Wrist.wristState.WALL);
                    slides.setTargetPos(Slides.GROUND);
                    setPathState(12);
                }
                break;
            case 12: // grab clip #3
                if (!follower.isBusy()) { /// make sure it works
                    claw.setState(Claw.ClawState.CLOSE);
                    setPathState(13);
                }
                break;
            case 13: // go to clip #3
                if (pathTime.getElapsedTimeSeconds() > 0.35) {
                    bar.setState(Bar.BarState.CLIP);
                    wrist.setState(Wrist.wristState.CLIP);
                    slides.setTargetPos(Slides.MED);
                    follower.setMaxPower(1);
                    follower.followPath(score3);
                    setPathState(14);
                }
                break;
            case 14: // let go of clip
                if (Math.abs(follower.getPose().getX() - SCORE3POSE.getX()) < 5){
                    claw.setState(Claw.ClawState.OPEN);
                    follower.followPath(score3ToWall);
                    setPathState(15);
                }
                break;
            case 15: // go to grab clip #4
                if (follower.getCurrentTValue() > 0.25) {
                    bar.setState(Bar.BarState.WALL);
                    wrist.setState(Wrist.wristState.WALL);
                    slides.setTargetPos(Slides.GROUND);
                    setPathState(16);
                }
                break;
            case 16: // grab #4
                if (!follower.isBusy()) { /// make sure it works
                    claw.setState(Claw.ClawState.CLOSE);
                    setPathState(17);
                }
                break;
            case 17: // go to clip #4
                if (pathTime.getElapsedTimeSeconds() > 0.35) {
                    bar.setState(Bar.BarState.CLIP);
                    wrist.setState(Wrist.wristState.CLIP);
                    slides.setTargetPos(Slides.MED);
                    follower.setMaxPower(1);
                    follower.followPath(score4);
                    setPathState(18);
                }
                break;
            case 18: // let go of clip
                if (Math.abs(follower.getPose().getX() - SCORE4POSE.getX()) < 5){
                    claw.setState(Claw.ClawState.OPEN);
                    follower.followPath(park);
                    setPathState(19);
                }
                break;
            case 19: // return & get ready for teleop
                if (follower.getCurrentTValue() > 0.25) {
                    bar.setState(Bar.BarState.PARK);
                    wrist.setState(Wrist.wristState.PARK);
                    slides.setTargetPos(Slides.GROUND);
                    extendo.setTargetPos(Extendo.MAX);
                    setPathState(-1);
                }
                break;
        }
    }
    private void setPathState(int n) {
        pathState = n;
        pathTime.resetTimer();
    }
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pathTime = new Timer();
        totalTime = new Timer();
        totalTime.resetTimer();
        loopTime = new Timer();
        loopTime.resetTimer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(STARTPOSE);
        follower.setMaxPower(0.85);
        buildPaths();

        bar = new Bar();
        claw = new Claw();
        extendo = new Extendo();
        intake = new Intake();
        intakeWrist = new IntakeWrist();
        slides = new Slides();
        wrist = new Wrist();

        led = hardwareMap.get(Servo.class, "shiny");
        bar.init(hardwareMap);
        claw.init(hardwareMap);
        extendo.init(hardwareMap);
        intake.init(hardwareMap);
        intakeWrist.init(hardwareMap);
        slides.init(hardwareMap);
        wrist.init(hardwareMap);
        slides.DANGEROUS_RESET_ENCODERS();
        extendo.DANGEROUS_RESET_ENCODERS();
        bar.setState(Bar.BarState.AUTOINIT);
        wrist.setState(Wrist.wristState.AUTOINIT);
        claw.setState(Claw.ClawState.CLOSE);
        claw.Loop(); //update Position
        bar.Loop();
        wrist.Loop();


    }

    double currentVoltage = 13;
    @Override
    public void init_loop() {
        currentVoltage += gamepad1.left_stick_y*0.006;
        telemetry.addData("Use gp1 left stick y", "\nto change current voltage for voltage compensation.");
        telemetry.addData("Current voltage: ", currentVoltage);
        extendo.Loop(currentVoltage);
        slides.Loop(currentVoltage);
        telemetry.update();
    }
    @Override
    public void start() {
        follower.drawOnDashBoard();
        led.setPosition(0.722);
        totalTime.resetTimer();
        loopTime.resetTimer();
        setPathState(0);
    }
    @Override
    public void loop() {
        follower.update();
        follower.drawOnDashBoard();
        updatePaths();
        bar.Loop();
        claw.Loop();
        extendo.Loop(currentVoltage);
        intake.Loop();
        intakeWrist.Loop();
        slides.Loop(currentVoltage);
        wrist.Loop();
        telemetry.addData("Loops per second", 1 / loopTime.getElapsedTimeSeconds());
        telemetry.addData("path state", pathState);
        telemetry.addData("Elapsed Time", pathTime.getElapsedTimeSeconds());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("claw state: ", claw.currentState);
        telemetry.addData("slides L/R ", slides.getLPos() + " / " + slides.getRPos());
        telemetry.update();
        loopTime.resetTimer();
    }

}
