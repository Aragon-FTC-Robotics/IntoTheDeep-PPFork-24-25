import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Disabled
@Autonomous(name = "3 Clip normal wall pickup (CLIP!)", group = "Auto")
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

    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;

    //Clip x: 41.29411764705882
    public static final Pose STARTPOSE = new Pose(6.75, 60, Math.toRadians(180));

    public static final Pose PRELOADPOSE = new Pose(49.923, 67.038, Math.toRadians(180));
    public static final Pose PREPARE1POSE = new Pose(57, 34, Math.toRadians(180));
    public static final Pose PREPARE1CONTROL = new Pose(11, 67);
    public static final Pose PREPARE1CONTROL2 = new Pose(44, 6);
    public static final Pose PREPARE1CONTROL3 = new Pose(60, 50);

    public static final Pose PUSH1POSE = new Pose(7, -49.633+72, Math.toRadians(180));

    public static final Pose PREPARE2POSE = new Pose(-17.629+72, 14.7, Math.toRadians(180));
    public static final Pose PREPARE2CONTROL = new Pose(75.176,30.882);

    public static final Pose PUSH2POSE = new Pose(6.3, 14.7, Math.toRadians(180));

    public static final Pose PREPARE3POSE = new Pose(-13.878+72, -62.211+72, Math.toRadians(0));
    public static final Pose PREPARE3CONTROL = new Pose(88.235,15, Math.toRadians(0));

    public static final Pose PUSH3POSE = new Pose(-54.078+72, -35.570+72, Math.toRadians(0));
    public static final Pose PUSH3CONTROL = new Pose(-2, 8);
    public static final Pose PUSH3CONTROL1 = new Pose(22, 7);
    public static final Pose PUSH3TOWALLCONTROL = new Pose(74.322, 36.427);
    public static final Pose PUSH2TOWALLCONTROL = new Pose(50,24); //old 55

    public static final Pose WALLPOSE = new Pose(12, 24, Math.toRadians(180));
    public static final Pose SCORE1POSE = new Pose(-26.9+72, 0.6+72, Math.toRadians(0));
    public static final Pose SCOREPOSECONTROL = new Pose(19, 36);
    public static final Pose SCOREPOSECONTROL2 = new Pose(14, 94);
    public static final Pose SCORETOWALLCONTROL = new Pose(23.033, 78.722);
    public static final Pose SCORETOWALLCONTROL2 = new Pose(40.678, 26.667); //changed

    public static final Pose SCORE2POSE = new Pose(-28.9+72, 0.6+72+2, Math.toRadians(180));
    public static final Pose SCORE3POSE = new Pose(-28.9+72, 0.677+72+4, Math.toRadians(180));
    public static final Pose SCORE4POSE = new Pose(-28.9+72, 0.677+72+6, Math.toRadians(180));
    public static final Pose PARKPOSE = new Pose(11.396, 11.882, Math.toRadians(0));
    public static final Pose PARKCONTROL = new Pose(7.235, 70.235);


    private PathChain scorePreload, prepare1, push1, prepare2, push2, prepare3, push3, push3ToWall, score1, score1ToWall, score2, score2ToWall, score3, score3ToWall, score4, park, push2ToWall; //Define paths

    private void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(STARTPOSE), new Point(PRELOADPOSE))))
                .setLinearHeadingInterpolation(STARTPOSE.getHeading(), PRELOADPOSE.getHeading())
                .build();
        prepare1 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(PRELOADPOSE), new Point(PREPARE1CONTROL), new Point(PREPARE1CONTROL2), new Point(PREPARE1CONTROL3), new Point(PREPARE1POSE))))
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
                .addPath(new Path(new BezierCurve(new Point(PREPARE3POSE), new Point(PUSH3CONTROL), new Point(PUSH3CONTROL1), new Point(PUSH3POSE))))
                .setLinearHeadingInterpolation(PREPARE3POSE.getHeading(), PUSH3POSE.getHeading())
                .build();
        push3ToWall = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(PUSH3POSE), new Point(PUSH3TOWALLCONTROL), new Point(WALLPOSE))))
                .setLinearHeadingInterpolation(PUSH3POSE.getHeading(), WALLPOSE.getHeading())
                .build();
        push2ToWall = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(PUSH2POSE), new Point(PUSH2TOWALLCONTROL), new Point(WALLPOSE))))
                .setLinearHeadingInterpolation(PUSH2POSE.getHeading(), WALLPOSE.getHeading())
                .build();
        score1 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(PUSH2POSE), new Point(SCOREPOSECONTROL), new Point(SCOREPOSECONTROL2), new Point(SCORE1POSE))))
                .setLinearHeadingInterpolation(PUSH2POSE.getHeading(), SCORE1POSE.getHeading())
                .build();
        score1ToWall = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(SCORE1POSE), new Point(SCORETOWALLCONTROL), new Point(SCORETOWALLCONTROL2), new Point(WALLPOSE))))
                .setLinearHeadingInterpolation(SCORE1POSE.getHeading(), WALLPOSE.getHeading())
                .build();
        score2 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(WALLPOSE), new Point(SCOREPOSECONTROL), new Point(SCOREPOSECONTROL2), new Point(SCORE2POSE))))
                .setLinearHeadingInterpolation(WALLPOSE.getHeading(), SCORE2POSE.getHeading())
                .build();
        score2ToWall = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(SCORE2POSE), new Point(SCORETOWALLCONTROL), new Point(SCORETOWALLCONTROL2), new Point(WALLPOSE))))
                .setLinearHeadingInterpolation(SCORE2POSE.getHeading(), WALLPOSE.getHeading())
                .build();
        score3 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(WALLPOSE), new Point(SCOREPOSECONTROL), new Point(SCOREPOSECONTROL2), new Point(SCORE3POSE))))
                .setLinearHeadingInterpolation(WALLPOSE.getHeading(), SCORE3POSE.getHeading())
                .build();
        score3ToWall = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(SCORE3POSE), new Point(SCORETOWALLCONTROL), new Point(SCORETOWALLCONTROL2), new Point(WALLPOSE))))
                .setLinearHeadingInterpolation(SCORE3POSE.getHeading(), WALLPOSE.getHeading())
                .build();
        score4 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(WALLPOSE), new Point(SCOREPOSECONTROL), new Point(SCOREPOSECONTROL2), new Point(SCORE4POSE))))
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
                extendo.setTargetPos(-200);
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
                if (pathTime.getElapsedTimeSeconds()>0.5) {
                    slides.setTargetPos(Slides.GROUND);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTime.getElapsedTimeSeconds()>0.4) {
                    claw.setState(Claw.ClawState.SUPEROPEN);
                    follower.followPath(prepare1, false);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(push1, false);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {  //Math.abs(follower.getPose().getX()-PUSH1POSE.getX())<1&&Math.abs(follower.getPose().getY()-PUSH1POSE.getY())<1
                    follower.followPath(prepare2, false);
                    bar.setState(Bar.BarState.WALL);
                    wrist.setState(Wrist.wristState.WALL);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) { //Math.abs(follower.getPose().getX()-PREPARE2POSE.getX())<1&&Math.abs(follower.getPose().getY()-PREPARE2POSE.getY())<1
                    follower.followPath(push2, true);
                    setPathState(1001);
                }
                break;
            case 6: //SKIPPED
                if (!follower.isBusy()) { //fix time
                    follower.followPath(push2ToWall, true);
                    setPathState(6001);
                }
                break;
            case 6001:
                if (pathTime.getElapsedTimeSeconds()>0.5) {
                    Log.d("I like to play", "with toys");
                    setPathState(1001);
                }
                break;
            case 1001:
                if (!follower.isBusy() || pathTime.getElapsedTimeSeconds()>2.5){
                    claw.setState(Claw.ClawState.CLOSE);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTime.getElapsedTimeSeconds()>0.5) {
                    slides.setTargetPos(Slides.MED);
                    setPathState(1101);
                }
                break;
            case 1101:
                if (pathTime.getElapsedTimeSeconds()>0.75) {
                    follower.followPath(score1);
                    setPathState(1102);
                }
                break;
            case 1102:
                if (pathTime.getElapsedTimeSeconds()>0.75) {
                    bar.setState(Bar.BarState.AUTOCLIP);
                    wrist.setState(Wrist.wristState.AUTOCLIP);
                    setPathState(12);
                }
                break;
            case 12:
                if (follower.isRobotStuck() || pathTime.getElapsedTimeSeconds()>3.5) {
                    slides.setTargetPos(Slides.GROUND);
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTime.getElapsedTimeSeconds()>0.22) {
                    claw.setState(Claw.ClawState.SUPEROPEN);
                    setPathState(1301);
                }
                break;
            case 1301:
                if (pathTime.getElapsedTimeSeconds() > 0.25) {
                    follower.followPath(score1ToWall, true);
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.WALL);
                    wrist.setState(Wrist.wristState.WALL);
                    setPathState(1401);
                }
                break;
            case 1401:
                if (!follower.isBusy()||pathTime.getElapsedTimeSeconds()>2.5){
                    claw.setState(Claw.ClawState.CLOSE);
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTime.getElapsedTimeSeconds()>0.5) {
                    slides.setTargetPos(Slides.MED);
                    setPathState(1501);
                }
                break;
            case 1501:
                if (pathTime.getElapsedTimeSeconds() > 0.75) {
                    follower.followPath(score2);
                    setPathState(1502);
                }
                break;
            case 1502:
                if (pathTime.getElapsedTimeSeconds() > 0.75) {
                    bar.setState(Bar.BarState.AUTOCLIP);
                    wrist.setState(Wrist.wristState.AUTOCLIP);
                    setPathState(16);
                }
                break;
            case 16:
                if (follower.isRobotStuck() || pathTime.getElapsedTimeSeconds()>2.5) {
                    slides.setTargetPos(Slides.GROUND);
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTime.getElapsedTimeSeconds() > 0.4) {
                    claw.setState(Claw.ClawState.SUPEROPEN);
                    setPathState(1701);
                }
                break;
            case 1701:
                if (pathTime.getElapsedTimeSeconds() > 0.25) {
                    follower.followPath(score2ToWall, true);
                    setPathState(18);
                }
                break;
            case 18:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.WALL);
                    wrist.setState(Wrist.wristState.WALL);
                    setPathState(1801);
                }
                break;
            case 1801:
                if (!follower.isBusy()||pathTime.getElapsedTimeSeconds()>3){
                    claw.setState(Claw.ClawState.CLOSE);
                    setPathState(19);
                }
                break;
            case 19:
                if (pathTime.getElapsedTimeSeconds()>0.5) {
                    slides.setTargetPos(Slides.MED);
                    setPathState(1901);
                }
                break;
            case 1901:
                if (pathTime.getElapsedTimeSeconds() > 0.75) {
                    follower.followPath(score3);
                    setPathState(1902);
                }
                break;
            case 1902:
                if (pathTime.getElapsedTimeSeconds() > 0.75) {
                    bar.setState(Bar.BarState.AUTOCLIP);
                    wrist.setState(Wrist.wristState.AUTOCLIP);
                    setPathState(20);
                }
            case 20:
                if (follower.isRobotStuck() || pathTime.getElapsedTimeSeconds()>3.5) {
                    slides.setTargetPos(Slides.GROUND);
                    setPathState(21);
                }
                break;
            case 21:
                if (pathTime.getElapsedTimeSeconds() > 0.4) {
                    claw.setState(Claw.ClawState.OPEN);
                    setPathState(2101);
                }
                break;
            case 2101:
                if (pathTime.getElapsedTimeSeconds() > 0.25) {
                    follower.followPath(park, true);
                    setPathState(26);
                }
                break;
//            case 22:
//                if (pathTime.getElapsedTimeSeconds() > 0.5) {
//                    bar.setState(Bar.BarState.WALL);
//                    wrist.setState(Wrist.wristState.WALL);
//                    setPathState(2201);
//                }
//                break;
//            case 2201:
//                if (!follower.isBusy()||pathTime.getElapsedTimeSeconds()>3.5){
//                    claw.setState(Claw.ClawState.CLOSE);
//                    setPathState(23);
//                }
//                break;
//            case 23:
//                if (pathTime.getElapsedTimeSeconds()>0.5) {
//                    slides.setTargetPos(Slides.MED);
//                    setPathState(2301);
//                }
//                break;
//            case 2301:
//                if (pathTime.getElapsedTimeSeconds() > 0.75) {
//                    follower.followPath(score4);
//                    setPathState(2302);
//                }
//                break;
//            case 2302:
//                if (pathTime.getElapsedTimeSeconds() > 0.75) {
//                    bar.setState(Bar.BarState.CLIP);
//                    wrist.setState(Wrist.wristState.AUTOCLIP);
//                    setPathState(24);
//                }
//            case 24:
//                if (!follower.isBusy()) {
//                    slides.setTargetPos(Slides.GROUND);
//                    setPathState(25);
//                }
//                break;
//            case 25:
//                if (pathTime.getElapsedTimeSeconds() > 0.22) {
//                    claw.setState(Claw.ClawState.OPEN);
//                    follower.followPath(park, true);
//                    setPathState(26);
//                }
//                break;
            case 26:
                if (pathTime.getElapsedTimeSeconds() > 0.5) { //set bar wrist to a init-able position
                    bar.setState(Bar.BarState.WALL);
                    wrist.setState(Wrist.wristState.WALL);
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

        bar.init(hardwareMap);
        claw.init(hardwareMap);
        extendo.init(hardwareMap);
        intake.init(hardwareMap);
        intakeWrist.init(hardwareMap);
        slides.init(hardwareMap);
        wrist.init(hardwareMap);
        slides.DANGEROUS_RESET_ENCODERS();
        extendo.DANGEROUS_RESET_ENCODERS();
        bar.setState(Bar.BarState.WALL);
        wrist.setState(Wrist.wristState.WALL);
        claw.setState(Claw.ClawState.CLOSE);
        claw.Loop(); //update Position
        bar.Loop();
        wrist.Loop();

        poseUpdater = new PoseUpdater(hardwareMap);

        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
        poseUpdater.setStartingPose(STARTPOSE);

    }

    double currentVoltage = 13;
    @Override
    public void init_loop() {
        currentVoltage += gamepad1.left_stick_y*0.006;
        telemetry.addData("Use gp1 left stick y", "\nto change current voltage for voltage compensation.");
        telemetry.addData("Current voltage: ", currentVoltage);
        telemetry.update();
    }
    @Override
    public void start() {
        totalTime.resetTimer();
        setPathState(0);
    }
    @Override
    public void loop() {
        follower.update();
        poseUpdater.update();
        updatePaths();
        bar.Loop();
        claw.Loop();
        extendo.Loop(currentVoltage);
        intake.Loop();
        intakeWrist.Loop();
        slides.Loop(currentVoltage);
        wrist.Loop();
        telemetry.addData("path state", pathState);
        telemetry.addData("Elapsed Time", pathTime.getElapsedTimeSeconds());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("slides L/R ", slides.getLPos() + " / " + slides.getRPos());
        telemetry.update();
        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }
}
