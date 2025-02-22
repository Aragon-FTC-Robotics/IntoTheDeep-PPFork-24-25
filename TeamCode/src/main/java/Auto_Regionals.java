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

@Autonomous(name = "AUTO clip pushing + relocalization", group = "Auto")
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
    private static final Pose PRELOADPOSE = new Pose(39, 67, Math.toRadians(180));
    private static final Pose PREPARE1POSE = new Pose(57,34,Math.toRadians(180));
    private static final Pose PREPARE1CONTROL = new Pose(19, 69);
    private static final Pose PREPARE1CONTROL2 = new Pose(24, 28);
    private static final Pose PUSH1POSE = new Pose(17,23,Math.toRadians(180));
    private static final Pose PUSH1CONTROL = new Pose(65,16);
    private static final Pose PREPARE2POSE = new Pose(54, 14.7, Math.toRadians(180));;
    private static final Pose PREPARE2CONTROL = new Pose(68,32);
    private static final Pose PUSH2POSE = new Pose(20, 17, Math.toRadians(180));
    private static final Pose PREPARE3POSE = new Pose(56, 9.5, Math.toRadians(180));
    private static final Pose PREPARE3CONTROL = new Pose(68, 22);
    private static final Pose PUSH3POSE = new Pose(7, 9.5, Math.toRadians(180));
    private static final Pose SCORE1MID = new Pose(12, 33, Math.toRadians(240));
    //Push3 -> score1 mid -> score1pose (with score1control)
    private static final Pose SCORE1CONTROL = new Pose(22, 77);
    private static final Pose SCORE1POSE = new Pose(50, 72.6, Math.toRadians(180)); //x was 45
    private static final Pose WALLPOSE = new Pose(7.8, 24, Math.toRadians(180));
    private static final Pose SCORECONTROL = new Pose(19, 36);
    private static final Pose SCORECONTROL2 = new Pose(14, 94);
    private static final Pose SCORETOWALLCONTROL = new Pose(100 - (29), 67); //was 16    or    score (45) - 16
    private static final Pose SCORETOWALLCONTROL2 = new Pose(100 - (9), 17); // was 36
    private static final Pose SCORE2POSE = new Pose(102, 72.6+2, Math.toRadians(180)); //x was 45
    private static final Pose SCORE3POSE =new Pose(102, 72.6+4, Math.toRadians(180)); //x was 45
    private static final Pose SCORE4POSE = new Pose(102, 72.6+6, Math.toRadians(180)); //x was 45
//    private static final Pose PARKCONTROL = new Pose(23, 82);
    private static final Pose PARK = new Pose(100 - 38, 16, Math.toRadians(-90));

    private static final Pose GOHOME = new Pose(36,73, Math.toRadians(180)); //UNUSED
    private static final Pose AFTERCLIPSCORE1POSE = new Pose(100, 72.6, Math.toRadians(180));
    private static final Pose AFTERCLIPSCORE2POSE = new Pose(100, 72.6 + 2, Math.toRadians(180));
    private static final Pose AFTERCLIPSCORE3POSE = new Pose(100, 72.6 + 4, Math.toRadians(180));
    private static final Pose AFTERCLIPSCORE4POSE = new Pose(100, 72.6 + 6, Math.toRadians(180));
    private static final Pose AFTERCLIPWALL = new Pose(100 - (37.2) + 15, 24, Math.toRadians(180));

    private static final Pose AFTERSCORETOWALLCONTROL = new Pose(100 - (29), 67);
    private static final Pose AFTERSCORETOWALLCONTROL2 = new Pose(100 - (5), 17);

    private static final Pose AFTERSCORECONTROL = new Pose(100 - (5), 17); //45 (score) -
    private static final Pose AFTERSCORECONTROL2 = new Pose(100 - (29), 67);

    private PathChain scorePreload, pushSample1, pushSample2, pushSample3, score1, score1ToWall, score2, score2ToWall, score3, score3ToWall, score4, park, gohome;

    private void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(STARTPOSE), new Point(PRELOADPOSE))))
                .setLinearHeadingInterpolation(STARTPOSE.getHeading(), PRELOADPOSE.getHeading())
                .build();
        pushSample1 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(PRELOADPOSE), new Point(PREPARE1CONTROL), new Point(PREPARE1CONTROL2), new Point(PREPARE1POSE))))
                .setConstantHeadingInterpolation(PRELOADPOSE.getHeading())
                .addPath(new Path(new BezierCurve(new Point(PREPARE1POSE), new Point(PUSH1CONTROL), new Point(PUSH1POSE))))
                .setConstantHeadingInterpolation(PRELOADPOSE.getHeading())
                .build();
        pushSample2 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(PUSH1POSE), new Point(PREPARE2CONTROL), new Point(PREPARE2POSE))))
                .setConstantHeadingInterpolation(PRELOADPOSE.getHeading())
                .addPath(new Path(new BezierLine(new Point(PREPARE2POSE), new Point(PUSH2POSE))))
                .setConstantHeadingInterpolation(PRELOADPOSE.getHeading())
                .build();
        pushSample3 = follower.pathBuilder()
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
                .addPath(new Path(new BezierCurve(new Point(AFTERCLIPSCORE1POSE), new Point(AFTERSCORETOWALLCONTROL), new Point(AFTERSCORETOWALLCONTROL2), new Point(AFTERCLIPWALL))))
                .setLinearHeadingInterpolation(AFTERCLIPSCORE1POSE.getHeading(), AFTERCLIPWALL.getHeading())
                .build();
        score2 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(WALLPOSE), new Point(AFTERSCORECONTROL), new Point(AFTERSCORECONTROL2), new Point(SCORE2POSE))))
                .setLinearHeadingInterpolation(WALLPOSE.getHeading(), SCORE2POSE.getHeading())
                .build();
        score2ToWall = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(AFTERCLIPSCORE2POSE), new Point(SCORETOWALLCONTROL), new Point(SCORETOWALLCONTROL2), new Point(AFTERCLIPWALL))))
                .setLinearHeadingInterpolation(SCORE2POSE.getHeading(), WALLPOSE.getHeading())
                .build();
        score3 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(WALLPOSE), new Point(AFTERSCORECONTROL), new Point(AFTERSCORECONTROL2), new Point(SCORE3POSE))))
                .setLinearHeadingInterpolation(WALLPOSE.getHeading(), SCORE3POSE.getHeading())
                .build();
        score3ToWall = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(AFTERCLIPSCORE3POSE), new Point(SCORETOWALLCONTROL), new Point(SCORETOWALLCONTROL2), new Point(AFTERCLIPWALL))))
                .setLinearHeadingInterpolation(SCORE3POSE.getHeading(), WALLPOSE.getHeading())
                .build();
        score4 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(WALLPOSE), new Point(AFTERSCORECONTROL), new Point(AFTERSCORECONTROL2), new Point(SCORE4POSE))))
                .setLinearHeadingInterpolation(WALLPOSE.getHeading(), SCORE3POSE.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(AFTERCLIPSCORE4POSE), new Point(PARK))))
                .setLinearHeadingInterpolation(SCORE4POSE.getHeading(), PARK.getHeading())
                .build();

        gohome = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(GOHOME), new Point(SCORETOWALLCONTROL), new Point(SCORETOWALLCONTROL2), new Point(AFTERCLIPWALL))))
                .setLinearHeadingInterpolation(GOHOME.getHeading(), WALLPOSE.getHeading())
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
                if (Math.abs(follower.getPose().getX() - PRELOADPOSE.getX()) < 4.5) {
                    bar.setState(Bar.BarState.DTCLIP2);
                    follower.followPath(pushSample1);
                    setPathState(2);
                }
                break;
            case 2: //open claw
                if (Math.abs(follower.getPose().getX() - PRELOADPOSE.getX()) > 7.5) { /// find exact timing
                    claw.setState(Claw.ClawState.OPEN);
                    setPathState(3);
                }
                break;
            case 3: // go to push samples
                if(follower.getCurrentTValue() > 0.2) {
                    bar.setState(Bar.BarState.WALL);
                    wrist.setState(Wrist.wristState.WALL);
                    setPathState(301);
                }
                break;
            case 301:
                if (follower.atParametricEnd() && follower.getCurrentPathNumber() > 0){
                    follower.followPath(pushSample2);
                    setPathState(302);
                }
                break;
            case 302:
                if (follower.atParametricEnd() && follower.getCurrentPathNumber() > 0){
                    follower.followPath(pushSample3);
                    setPathState(4);
                }
            case 4: // grab score 1 (technically 2)
                if (!follower.isBusy()) {
                    claw.setState(Claw.ClawState.CLOSE);
                    setPathState(5);
                }
                break;
            case 5: // get into position/move to bar/clip
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.DTCLIP1);
                    wrist.setState(Wrist.wristState.DTCLIP);
                    follower.followPath(score1);
                    setPathState(6);
                }
                break;
            case 6: //let go of clip after clipped
                if (follower.isRobotStuck() || !follower.isBusy()){
                    bar.setState(Bar.BarState.DTCLIP2);
                    follower.breakFollowing();
                    follower.setPose(new Pose(100, follower.getPose().getY(), follower.getPose().getHeading()));
                    follower.followPath(score1ToWall);
                    setPathState(601);
                }
                break;
            case 601:
                if (follower.getCurrentTValue() > 0.1) {
                    claw.setState(Claw.ClawState.OPEN);
                    setPathState(7);
                }
                break;
            case 7: // get ready to grab 2nd clip
                if (pathTime.getElapsedTimeSeconds() > 0.75) {
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
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.DTCLIP1);
                    wrist.setState(Wrist.wristState.DTCLIP);
                    follower.followPath(score2);
                    setPathState(10);
                }
                break;
            case 10: // let go of clip
                if (follower.isRobotStuck() || !follower.isBusy()){
                    follower.breakFollowing();
                    follower.setPose(new Pose(100, follower.getPose().getY(), follower.getPose().getHeading()));
                    bar.setState(Bar.BarState.DTCLIP2);
                    follower.followPath(score2ToWall);
                    setPathState(1001);
                }
                break;
            case 1001:
                if(follower.getCurrentTValue() > 0.03){
                    claw.setState(Claw.ClawState.OPEN);
                    setPathState(11);
                }
                break;
            case 11: // get ready for clip #3
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.WALL);
                    wrist.setState(Wrist.wristState.WALL);
                    slides.setTargetPos(Slides.GROUND);
                    setPathState(12);
                }
                break;
            case 12: // grab clip #3
                if (!follower.isBusy()) { // make sure it works
                    claw.setState(Claw.ClawState.CLOSE);
                    setPathState(13);
                }
                break;
            case 13: // go to clip #3
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.DTCLIP1);
                    wrist.setState(Wrist.wristState.DTCLIP);
                    follower.followPath(score3);
                    setPathState(14);
                }
                break;
            case 14: // let go of clip
                if (follower.isRobotStuck() || !follower.isBusy()){
                    follower.breakFollowing();
                    follower.setPose(new Pose(100, follower.getPose().getY(), follower.getPose().getHeading()));
                    bar.setState(Bar.BarState.DTCLIP2);
                    follower.followPath(score3ToWall);
                    setPathState(1401);
                }
                break;
            case 1401:
                if (follower.getCurrentTValue() > 0.03) {
                    claw.setState(Claw.ClawState.OPEN);
                    setPathState(15);
                }
                break;
            case 15: // go to grab clip #4
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.WALL);
                    wrist.setState(Wrist.wristState.WALL);
                    setPathState(16);
                }
                break;
            case 16: // grab #4
                if (!follower.isBusy()) { // make sure it works
                    claw.setState(Claw.ClawState.CLOSE);
                    setPathState(17);
                }
                break;
            case 17: // go to clip #4
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.DTCLIP1);
                    wrist.setState(Wrist.wristState.DTCLIP);
                    follower.followPath(score4);
                    setPathState(18);
                }
                break;
            case 18: // let go of clip
                if (follower.isRobotStuck() || !follower.isBusy()){
                    follower.breakFollowing();
                    follower.setPose(new Pose(100, follower.getPose().getY(), follower.getPose().getHeading()));
                    bar.setState(Bar.BarState.DTCLIP2);
                    follower.followPath(park);
                    setPathState(1801);
                }
                break;
            case 1801:
                if(follower.getCurrentTValue() > 0.03) {
                    claw.setState(Claw.ClawState.OPEN);
                    setPathState(19);
                }
                break;
            case 19: // return & get ready for teleop
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.PARK);
                    wrist.setState(Wrist.wristState.PARK);
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
        follower.setMaxPower(0.8);
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

        follower.update();
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
        telemetry.addData("path number", follower.getCurrentPathNumber());
        telemetry.addData("T value", follower.getCurrentTValue());
        telemetry.addData("Elapsed Time", pathTime.getElapsedTimeSeconds());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("claw state: ", claw.currentState);
        telemetry.addData("slides L/R ", slides.getLPos() + " / " + slides.getRPos());
        telemetry.update();
        loopTime.resetTimer();
        follower.telemetryDebug(telemetry);
    }

}
