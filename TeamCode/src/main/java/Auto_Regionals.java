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

@Autonomous(name = "Hopefully better Clip Auto", group = "Auto")
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
    private Timer pathTime, totalTime;
    private int pathState = 0;

    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;

    private static final Pose STARTPOSE = Auto_5_0.STARTPOSE;
    private static final Pose PRELOADPOSE = Auto_5_0.PRELOADPOSE;
    private static final Pose PREPAREMID = new Pose(32,46, Math.toRadians(0));
    private static final Pose PREPAREMIDCONTROL = new Pose(31, 66);
    private static final Pose PREPARE1POSE = new Pose(57,34,Math.toRadians(180));
    private static final Pose PREPARE1CONTROL = new Pose(19, 69);
    private static final Pose PREPARE1CONTROL2 = new Pose(24, 28);
    private static final Pose PUSH1POSE = new Pose(-12,23,Math.toRadians(180));
    private static final Pose PUSH1CONTROL = new Pose(65,16);
    private static final Pose PREPARE2POSE = Auto_5_0.PREPARE2POSE;
    private static final Pose PREPARE2CONTROL = Auto_5_0.PREPARE2CONTROL;
    private static final Pose PUSH2POSE = Auto_5_0.PUSH2POSE;
    private static final Pose PUSH2MID = new Pose(12, 33, Math.toRadians(240));
    private static final Pose WALLPOSE = new Pose(8.3, 24, Math.toRadians(180));
    private static final Pose SCORE1POSE = new Pose(Auto_5_0.SCORE1POSE.getX(),Auto_5_0.SCORE1POSE.getY(),Math.toRadians(180));
    private static final Pose SCORE1CONTROL = new Pose(22, 77); //for scoring first sampel after moving out of observation
    private static final Pose SCOREPOSECONTROL = Auto_5_0.SCOREPOSECONTROL;
    private static final Pose SCOREPOSECONTROL2 = Auto_5_0.SCOREPOSECONTROL2;
    private static final Pose SCORETOWALLCONTROL = Auto_5_0.SCORETOWALLCONTROL;
    private static final Pose SCORETOWALLCONTROL2 = Auto_5_0.SCORETOWALLCONTROL2;
    private static final Pose SCORE2POSE = new Pose(Auto_5_0.SCORE2POSE.getX(),Auto_5_0.SCORE2POSE.getY(),Math.toRadians(180));
    private static final Pose SCORE3POSE = new Pose(Auto_5_0.SCORE3POSE.getX()+2,Auto_5_0.SCORE3POSE.getY(),Math.toRadians(180));
    private static final Pose SCORE4POSE = Auto_5_0.SCORE4POSE;
    private static final Pose PARKPOSE = Auto_5_0.PARKPOSE;
    private static final Pose PARKCONTROL = Auto_5_0.PARKCONTROL;


    private PathChain scorePreload, prepare1, push1, prepare2, push2, score1, score1ToWall, score2, score2ToWall, score3, score3ToWall, score4, score4ToWall, park;

    private void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(STARTPOSE), new Point(PRELOADPOSE))))
                .setLinearHeadingInterpolation(STARTPOSE.getHeading(), PRELOADPOSE.getHeading())
                .build();
        prepare1 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(PRELOADPOSE), new Point(PREPARE1CONTROL), new Point(PREPARE1CONTROL2), new Point(PREPARE1POSE))))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();
        push1 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(PREPARE1POSE), new Point(PUSH1CONTROL), new Point(PUSH1POSE))))
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
        score1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(PUSH2POSE), new Point(PUSH2MID))))
                .setLinearHeadingInterpolation(PUSH2POSE.getHeading(), PUSH2MID.getHeading())
                .addPath(new Path(new BezierCurve(new Point(PUSH2MID), new Point(SCORE1CONTROL), new Point(SCORE1POSE))))
                .setLinearHeadingInterpolation(PUSH2MID.getHeading(), SCORE1POSE.getHeading())
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
    }
    private void updatePaths() {
        switch (pathState) {
            case 0:
                extendo.setTargetPos(-100);
                intakeWrist.setState(IntakeWrist.intakeWristState.IN);
                slides.setTargetPos(900);
                bar.setState(Bar.BarState.DTFIRSTCLIP);
                wrist.setState(Wrist.wristState.DTFIRSTCLIP);
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;
            case 1:
                if ((Math.abs(follower.getPose().getX()-44.923)<4) || follower.isRobotStuck()) { //Wait for robot to reach clip position
                    Log.d("Hello!", "It's my birthday!");
                    setPathState(2);
                }
                break;
//            case 101:
//                if (pathTime.getElapsedTimeSeconds() > 0.05) {
//                    slides.setTargetPos(Slides.GROUND);
//                    setPathState(2);
//                }
//                break;
            case 2:
                if (true) {
                    slides.setTargetPos(Slides.GROUND);
                    claw.setState(Claw.ClawState.SUPEROPEN);
                    follower.followPath(prepare1, false);
                    setPathState(3);
                }
                break;
            case 3:
                if (Math.abs(follower.getPose().getX()-PREPARE1POSE.getX())<6&&Math.abs(follower.getPose().getY()-PREPARE1POSE.getY())<6) {
                    follower.followPath(push1, false);
                    setPathState(4);
                }
                break;
            case 4:
                if (Math.abs(follower.getPose().getX()-13)<6) {
                    follower.followPath(prepare2, false);
                    setPathState(5);
                }
                break;
            case 5:
                if (Math.abs(follower.getPose().getX()-PREPARE2POSE.getX())<3&&Math.abs(follower.getPose().getY()-PREPARE2POSE.getY())<3) {
                    follower.setMaxPower(0.85);
                    follower.followPath(push2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() || follower.isRobotStuck()) { //Todo IDK!!!!
                    claw.setState(Claw.ClawState.CLOSE);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTime.getElapsedTimeSeconds() > 0.35) {
                    bar.setState(Bar.BarState.CLIP);
                    wrist.setState(Wrist.wristState.CLIP);
                    slides.setTargetPos(Slides.MED);
                    follower.setMaxPower(1);
                    follower.followPath(score1);
                    setPathState(901);
                }
                break;
            case 901:
                if (pathTime.getElapsedTimeSeconds() > 0.05) {
                    follower.setMaxPower(0.85);
                    follower.followPath(score1ToWall, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTime.getElapsedTimeSeconds() > 0.1) { //Todo needs heavy tuning
                    claw.setState(Claw.ClawState.SUPEROPEN);
                    bar.setState(Bar.BarState.WALL);
                    wrist.setState(Wrist.wristState.WALL);
                    slides.setTargetPos(Slides.GROUND);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy() || follower.isRobotStuck()) { //Todo idk
                    claw.setState(Claw.ClawState.CLOSE);
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTime.getElapsedTimeSeconds() > 0.35) {
                    bar.setState(Bar.BarState.CLIP);
                    wrist.setState(Wrist.wristState.CLIP);
                    slides.setTargetPos(Slides.MED);
                    follower.setMaxPower(1);
                    follower.followPath(score2);
                    setPathState(1401);
                }
                break;
            case 1401:
                if (pathTime.getElapsedTimeSeconds() > 0.05) {
                    follower.setMaxPower(0.85);
                    follower.followPath(score2ToWall, true);
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTime.getElapsedTimeSeconds() > 0.1) { //Todo needs heavy tuning,,,,
                    claw.setState(Claw.ClawState.SUPEROPEN);
                    bar.setState(Bar.BarState.WALL);
                    wrist.setState(Wrist.wristState.WALL);
                    slides.setTargetPos(Slides.GROUND);
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy() || follower.isRobotStuck()) { //Todo IDK!!!
                    claw.setState(Claw.ClawState.CLOSE);
                    setPathState(1601);
                }
                break;
//            case 1601:
//                if(pathTime.getElapsedTimeSeconds() > 0.5) {
//                    bar.setState(Bar.BarState.DTWALLSILLY);
//                    wrist.setState(Wrist.wristState.DTWALLSILLY);
//                    setPathState(17);
//                }
//                break;
            case 17:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.CLIP);
                    wrist.setState(Wrist.wristState.CLIP);
                    follower.setMaxPower(1);
                    follower.followPath(score3);
                    setPathState(18);
                }
                break;
//            case 18:
//                if (pathTime.getElapsedTimeSeconds() > 0.5) {
//                    bar.setState(Bar.BarState.DTCLIP1);
//                    wrist.setState(Wrist.wristState.DTCLIP);
//                    setPathState(19);
//                }
//                break;
//            case 19:
//                if (!follower.isBusy() || follower.isRobotStuck()) {
//                    intakeWrist.setState(IntakeWrist.intakeWristState.IN);
//                    bar.setState(Bar.BarState.DTCLIP2);
//                    setPathState(1901);
//                }
//                break;
            case 1901:
                if (pathTime.getElapsedTimeSeconds() > 0.05) {
                    follower.followPath(score3ToWall, true);
                    setPathState(20);
                }
                break;
            case 20:
                if (pathTime.getElapsedTimeSeconds() > 0.1) { //Todo needs heavy tuning,,,,
                    claw.setState(Claw.ClawState.SUPEROPEN);
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
        follower.setMaxPower(1);
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
        bar.setState(Bar.BarState.WALL);
        wrist.setState(Wrist.wristState.WALL);
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
        setPathState(0);
    }
    @Override
    public void loop() {
        follower.update();
        follower.drawOnDashBoard();
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
        telemetry.addData("claw state: ", claw.currentState);
        telemetry.addData("slides L/R ", slides.getLPos() + " / " + slides.getRPos());
        telemetry.update();
    }

}
