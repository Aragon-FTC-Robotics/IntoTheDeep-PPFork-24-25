import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import mechanisms.Bar;
import mechanisms.Claw;
import mechanisms.Extendo;
import mechanisms.Intake;
import mechanisms.IntakeWrist;
import mechanisms.Slides;
import mechanisms.Wrist;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "5+0 Specimen", group = "Auto")
public class Auto_5_0 extends OpMode {
    private Bar bar;
    private Claw claw;
    private Extendo extendo;
    private Intake intake;
    private IntakeWrist intakeWrist;
    private Slides slides;
    private Wrist wrist;


    private Follower follower;
    private Timer pathTime, totalTime;
    private int pathState = 0;


    //Clip x: 41.29411764705882
    private static final Pose STARTPOSE = new Pose(-65.25+72, -12+72, Math.toRadians(0));
    private static final Pose PRELOADPOSE = new Pose(-30.467+72, -2.9+72, Math.toRadians(0));
    private static final Pose PREPARE1POSE = new Pose(-14.489+72, -46.716+72, Math.toRadians(0));
    private static final Pose PREPARE1CONTROL = new Pose(10.294, 57.647, Math.toRadians(0));
    private static final Pose PUSH1POSE = new Pose(-57.524+72, -43.633+72, Math.toRadians(0));
    private static final Pose PREPARE2POSE = new Pose(-17.629+72, -57.676+72, Math.toRadians(0));
    private static final Pose PREPARE2CONTROL = new Pose(75.176,30.882, Math.toRadians(0));
    private static final Pose PUSH2POSE = new Pose(-58.897+72, -57.339+72, Math.toRadians(0));
    private static final Pose PREPARE3POSE = new Pose(-13.878+72, -61.211+72, Math.toRadians(0));
    private static final Pose PREPARE3CONTROL = new Pose(82.235,15, Math.toRadians(0));
    private static final Pose PUSH3POSE = new Pose(-54.478+72, -35.570+72, Math.toRadians(0));
    private static final Pose PUSH3CONTROL = new Pose(6, 1.588, Math.toRadians(0));
    private static final Pose PUSH3TOWALLCONTROL = new Pose(66, 33);
    private static final Pose WALLPOSE = new Pose(-62.820+72, -44.185+72, Math.toRadians(180));
    private static final Pose SCORE1POSE = new Pose(-36+72, 1.6+72, Math.toRadians(0));
    private static final Pose SCORE2POSE = new Pose(-36+72, 1.6+72+2, Math.toRadians(0));
    private static final Pose SCORE3POSE = new Pose(-36+72, 1.677+72+4, Math.toRadians(0));
    private static final Pose SCORE4POSE = new Pose(-36+72, 1.677+72+6, Math.toRadians(0));
    private static final Pose PARKPOSE = new Pose(8.396, 6.882, Math.toRadians(-10));
    private static final Pose PARKCONTROL = new Pose(7.235, 70.235);


    private PathChain scorePreload, prepare1, push1, prepare2, push2, prepare3, push3, push3ToWall, score1, score1ToWall, score2, score2ToWall, score3, score3ToWall, score4, park; //Define paths

    private void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(STARTPOSE), new Point(PRELOADPOSE))))
                .setLinearHeadingInterpolation(STARTPOSE.getHeading(), PRELOADPOSE.getHeading())
                .build();
        prepare1 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(PRELOADPOSE), new Point(PREPARE1CONTROL), new Point(PREPARE1POSE))))
                .setLinearHeadingInterpolation(PRELOADPOSE.getHeading(), PREPARE1POSE.getHeading())
                .build();
        push1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(PREPARE1POSE), new Point(PUSH1POSE))))
                .setLinearHeadingInterpolation(PREPARE1POSE.getHeading(), PUSH1POSE.getHeading())
                .build();
        prepare2 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(PUSH1POSE), new Point(PREPARE2CONTROL), new Point(PREPARE2POSE))))
                .setLinearHeadingInterpolation(PUSH1POSE.getHeading(), PREPARE2POSE.getHeading())
                .build();
        push2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(PREPARE2POSE), new Point(PUSH2POSE))))
                .setLinearHeadingInterpolation(PREPARE2POSE.getHeading(), PUSH2POSE.getHeading())
                .build();
        prepare3 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(PUSH2POSE),new Point(PREPARE3CONTROL), new Point(PREPARE3POSE))))
                .setLinearHeadingInterpolation(PUSH2POSE.getHeading(), PREPARE3POSE.getHeading())
                .build();
        push3 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(PREPARE3POSE), new Point(PUSH3CONTROL), new Point(PUSH3POSE))))
                .setLinearHeadingInterpolation(PREPARE3POSE.getHeading(), PUSH3POSE.getHeading())
                .build();
        push3ToWall = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(PUSH3POSE), new Point(PUSH3TOWALLCONTROL), new Point(WALLPOSE))))
                .setLinearHeadingInterpolation(PUSH3POSE.getHeading(), WALLPOSE.getHeading())
                .build();
        score1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(WALLPOSE), new Point(SCORE1POSE))))
                .setLinearHeadingInterpolation(WALLPOSE.getHeading(), SCORE1POSE.getHeading())
                .build();
        score1ToWall = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(SCORE1POSE), new Point(WALLPOSE))))
                .setLinearHeadingInterpolation(SCORE1POSE.getHeading(), WALLPOSE.getHeading())
                .build();
        score2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(WALLPOSE), new Point(SCORE2POSE))))
                .setLinearHeadingInterpolation(WALLPOSE.getHeading(), SCORE2POSE.getHeading())
                .build();
        score2ToWall = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(SCORE2POSE), new Point(WALLPOSE))))
                .setLinearHeadingInterpolation(SCORE2POSE.getHeading(), WALLPOSE.getHeading())
                .build();
        score3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(WALLPOSE), new Point(SCORE3POSE))))
                .setLinearHeadingInterpolation(WALLPOSE.getHeading(), SCORE3POSE.getHeading())
                .build();
        score3ToWall = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(SCORE3POSE), new Point(WALLPOSE))))
                .setLinearHeadingInterpolation(SCORE3POSE.getHeading(), WALLPOSE.getHeading())
                .build();
        score4 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(WALLPOSE), new Point(SCORE4POSE))))
                .setLinearHeadingInterpolation(WALLPOSE.getHeading(), SCORE4POSE.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(SCORE4POSE), new Point(PARKCONTROL), new Point(PARKPOSE))))
                .setLinearHeadingInterpolation(SCORE4POSE.getHeading(), PARKPOSE.getHeading())
                .build();
    }
    private void updatePaths() {
        switch (pathState) {
            case 0:
                extendo.setTargetPos(Extendo.MIN);
                intakeWrist.setState(IntakeWrist.intakeWristState.IN);
                slides.setTargetPos(Slides.MED);
                bar.setState(Bar.BarState.CLIP);
                wrist.setState(Wrist.wristState.CLIP);
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    Log.d("Hello!", "It's my birthday!");
                    setPathState(101);
                }
                break;
            case 101:
                if (pathTime.getElapsedTimeSeconds()>1) {
                    slides.setTargetPos(Slides.LOW);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTime.getElapsedTimeSeconds()>0.22) {
                    claw.setState(Claw.ClawState.OPEN);
                    follower.followPath(prepare1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(push1, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(prepare2, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(push2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(prepare3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    wrist.setState(Wrist.wristState.WALL);
                    follower.followPath(push3, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    bar.setState(Bar.BarState.WALL);
                    follower.followPath(push3ToWall, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (true) {
                    Log.d("Hello!", "It's no longer my birthday!");
                    setPathState(10);
                }
                break;
            case 10:
                if (true) {
                    Log.d("Bye!", "I'm going to school!");
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    slides.setTargetPos(Slides.MED);
                    claw.setState(Claw.ClawState.CLOSE);
                    follower.followPath(score1);
                    setPathState(1101);
                }
                break;
            case 1101:
                if (pathTime.getElapsedTimeSeconds()>0.75) {
                    bar.setState(Bar.BarState.CLIP);
                    wrist.setState(Wrist.wristState.CLIP);
                    setPathState(12);
                }
                break;
            case 12:
                if ((!follower.isBusy())) {
                    slides.setTargetPos(Slides.LOW);
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTime.getElapsedTimeSeconds()>0.22) {
                    claw.setState(Claw.ClawState.OPEN);
                    follower.followPath(score1ToWall, true);
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.WALL);
                    wrist.setState(Wrist.wristState.WALL);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    slides.setTargetPos(Slides.MED);
                    claw.setState(Claw.ClawState.CLOSE);
                    follower.followPath(score2);
                    setPathState(1501);
                }
                break;
            case 1501:
                if (pathTime.getElapsedTimeSeconds() > 0.75) {
                    bar.setState(Bar.BarState.CLIP);
                    wrist.setState(Wrist.wristState.CLIP);
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    slides.setTargetPos(Slides.LOW);
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTime.getElapsedTimeSeconds() > 0.22) {
                    claw.setState(Claw.ClawState.OPEN);
                    follower.followPath(score2ToWall, true);
                    setPathState(18);
                }
                break;
            case 18:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.WALL);
                    wrist.setState(Wrist.wristState.WALL);
                    setPathState(19);
                }
                break;
            case 19:
                if (!follower.isBusy()) {
                    slides.setTargetPos(Slides.MED);
                    claw.setState(Claw.ClawState.CLOSE);
                    follower.followPath(score3);
                    setPathState(1901);
                }
                break;
            case 1901:
                if (pathTime.getElapsedTimeSeconds()>0.75) {
                    bar.setState(Bar.BarState.CLIP);
                    wrist.setState(Wrist.wristState.CLIP);
                    setPathState(20);
                }
                break;
            case 20:
                if (!follower.isBusy()) {
                    slides.setTargetPos(Slides.LOW);
                    setPathState(21);
                }
                break;
            case 21:
                if (pathTime.getElapsedTimeSeconds() > 0.22) {
                    claw.setState(Claw.ClawState.OPEN);
                    follower.followPath(score3ToWall, true);
                    setPathState(22);
                }
                break;
            case 22:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.WALL);
                    wrist.setState(Wrist.wristState.WALL);
                    setPathState(23);
                }
                break;
            case 23:
                if (!follower.isBusy()) {
                    slides.setTargetPos(Slides.MED);
                    claw.setState(Claw.ClawState.CLOSE);
                    follower.followPath(score4);
                    setPathState(2301);
                }
                break;
            case 2301:
                if (pathTime.getElapsedTimeSeconds()>0.75) {
                    bar.setState(Bar.BarState.CLIP);
                    wrist.setState(Wrist.wristState.CLIP);
                    setPathState(24);
                }
                break;
            case 24:
                if (!follower.isBusy()) {
                    slides.setTargetPos(Slides.LOW);
                    setPathState(25);
                }
                break;
            case 25:
                if (pathTime.getElapsedTimeSeconds() > 0.22) {
                    claw.setState(Claw.ClawState.OPEN);
                    follower.followPath(park, true);
                    setPathState(26);
                }
                break;
            case 26:
                if (pathTime.getElapsedTimeSeconds() > 0.5) { //set bar wrist to a init-able position
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
          pathTime = new Timer();
        totalTime = new Timer();
        totalTime.resetTimer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(STARTPOSE);
        follower.setMaxPower(0.7);
        buildPaths();

        bar = new Bar();
        claw = new Claw();
        extendo = new Extendo();
        intake = new Intake();
        intakeWrist = new IntakeWrist();
        slides = new Slides();
        wrist = new Wrist();

        bar.init(hardwareMap);
        claw.init(hardwareMap);
        extendo.init(hardwareMap);
        intake.init(hardwareMap);
        intakeWrist.init(hardwareMap);
        slides.init(hardwareMap);
        wrist.init(hardwareMap);

        bar.setState(Bar.BarState.NEUTRAL);
        wrist.setState(Wrist.wristState.NEUTRAL);
        claw.setState(Claw.ClawState.CLOSE);
        claw.Loop(); //update Position
        bar.Loop();
        wrist.Loop();

    }
    @Override
    public void start() {
        totalTime.resetTimer();
        setPathState(0);
    }
    @Override
    public void loop() {
        follower.update();
        updatePaths();
        bar.Loop();
        claw.Loop();
        extendo.Loop();
        intake.Loop();
        intakeWrist.Loop();
        slides.Loop();
        wrist.Loop();
        telemetry.addData("path state", pathState);
        telemetry.addData("Elapsed Time", pathTime.getElapsedTimeSeconds());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
