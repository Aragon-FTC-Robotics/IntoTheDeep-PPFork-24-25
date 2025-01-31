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

import mechanisms.Bar;
import mechanisms.Claw;
import mechanisms.Extendo;
import mechanisms.Intake;
import mechanisms.IntakeWrist;
import mechanisms.Slides;
import mechanisms.Wrist;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Chopped Chin Autonomous (GOOD CLIP!!!)", group = "Auto")
public class Auto_Silly_spec extends OpMode {
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

    private static final Pose STARTPOSE = Auto_5_0.STARTPOSE;
    private static final Pose PRELOADPOSE = Auto_5_0.PRELOADPOSE;
    private static final Pose PREPARE1POSE = Auto_5_0.PREPARE1POSE;
    private static final Pose PREPARE1CONTROL = Auto_5_0.PREPARE1CONTROL;
    private static final Pose PREPARE1CONTROL2 = Auto_5_0.PREPARE1CONTROL2;
    private static final Pose PUSH1POSE = Auto_5_0.PUSH1POSE;
    private static final Pose PREPARE2POSE = Auto_5_0.PREPARE2POSE;
    private static final Pose PREPARE2CONTROL = Auto_5_0.PREPARE2CONTROL;
    private static final Pose PUSH2POSE = Auto_5_0.PUSH2POSE;
    private static final Pose WALLPOSE = new Pose(7.8, 24, Math.toRadians(180));
    private static final Pose SCORE1POSE = new Pose(Auto_5_0.SCORE1POSE.getX(),Auto_5_0.SCORE1POSE.getY(),Math.toRadians(180));
    private static final Pose SCORE1POSECONTROL = Auto_5_0.SCORE1POSECONTROL;
    private static final Pose SCORE1POSECONTROL2 = Auto_5_0.SCORE1POSECONTROL2;
    private static final Pose SCORETOWALLCONTROL = Auto_5_0.SCORETOWALLCONTROL;
    private static final Pose SCORETOWALLCONTROL2 = Auto_5_0.SCORETOWALLCONTROL2;
    private static final Pose SCORE2POSE = new Pose(Auto_5_0.SCORE2POSE.getX(),Auto_5_0.SCORE2POSE.getY(),Math.toRadians(180));
    private static final Pose SCORE3POSE = new Pose(Auto_5_0.SCORE3POSE.getX(),Auto_5_0.SCORE3POSE.getY(),Math.toRadians(180));
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
        score1 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(PUSH2POSE), new Point(SCORE1POSECONTROL), new Point(SCORE1POSECONTROL2), new Point(SCORE1POSE))))
                .setLinearHeadingInterpolation(PUSH2POSE.getHeading(), SCORE1POSE.getHeading())
                .build();
        score1ToWall = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(SCORE1POSE), new Point(SCORETOWALLCONTROL), new Point(SCORETOWALLCONTROL2), new Point(WALLPOSE))))
                .setLinearHeadingInterpolation(SCORE1POSE.getHeading(), WALLPOSE.getHeading())
                .build();
        score2 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(WALLPOSE), new Point(SCORE1POSECONTROL), new Point(SCORE1POSECONTROL2), new Point(SCORE2POSE))))
                .setLinearHeadingInterpolation(WALLPOSE.getHeading(), SCORE2POSE.getHeading())
                .build();
        score2ToWall = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(SCORE2POSE), new Point(SCORETOWALLCONTROL), new Point(SCORETOWALLCONTROL2), new Point(WALLPOSE))))
                .setLinearHeadingInterpolation(SCORE2POSE.getHeading(), WALLPOSE.getHeading())
                .build();
        score3 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(WALLPOSE), new Point(SCORE1POSECONTROL), new Point(SCORE1POSECONTROL2), new Point(SCORE3POSE))))
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
                slides.setTargetPos(Slides.MED);
                bar.setState(Bar.BarState.CLIP);
                wrist.setState(Wrist.wristState.CLIP);
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) { //Wait for robot to reach clip position
                    Log.d("Hello!", "It's my birthday!");
                    setPathState(101);
                }
                break;
            case 101:
                if (pathTime.getElapsedTimeSeconds() > 0.2) {
                    slides.setTargetPos(Slides.GROUND);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTime.getElapsedTimeSeconds() > 0.28) { //Todo NEEDS HEAVY TUNING
                    claw.setState(Claw.ClawState.SUPEROPEN);
                    follower.followPath(prepare1, false);
                    setPathState(3);
                }
                break;
            case 3:
                if (Math.abs(follower.getPose().getX()-PREPARE1POSE.getX())<3&&Math.abs(follower.getPose().getY()-PREPARE1POSE.getY())<3) {
                    follower.followPath(push1, false);
                    setPathState(4);
                }
                break;
            case 4:
                if (Math.abs(follower.getPose().getX()-PUSH1POSE.getX())<3&&Math.abs(follower.getPose().getY()-PUSH1POSE.getY())<3) {
                    follower.followPath(prepare2, false);
                    bar.setState(Bar.BarState.DTWALL);
                    wrist.setState(Wrist.wristState.DTWALL);
                    intakeWrist.setState(IntakeWrist.intakeWristState.WALLALIGN);
                    setPathState(5);
                }
                break;
            case 5:
                if (Math.abs(follower.getPose().getX()-PREPARE2POSE.getX())<3&&Math.abs(follower.getPose().getY()-PREPARE2POSE.getY())<3) {
                    follower.followPath(push2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() || follower.isRobotStuck()) { //Todo IDK!!!!
                    claw.setState(Claw.ClawState.CLOSE);
                    setPathState(601);
                }
                break;
            case 601:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.DTWALLSILLY);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTime.getElapsedTimeSeconds() > 0.35) {
                    bar.setState(Bar.BarState.CLIP);
                    follower.followPath(score1);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.DTCLIP1);
                    wrist.setState(Wrist.wristState.DTCLIP);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy() || follower.isRobotStuck()) {
                    bar.setState(Bar.BarState.DTCLIP2);
                    setPathState(901);
                }
                break;
            case 901:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(score1ToWall, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTime.getElapsedTimeSeconds() > 0.4) { //Todo needs heavy tuning
                    claw.setState(Claw.ClawState.SUPEROPEN);
                    bar.setState(Bar.BarState.DTWALL);
                    wrist.setState(Wrist.wristState.DTWALL);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy() || follower.isRobotStuck()) { //Todo idk
                    claw.setState(Claw.ClawState.CLOSE);
                    setPathState(1101);
                }
                break;
            case 1101:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.DTWALLSILLY);
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTime.getElapsedTimeSeconds() > 0.35) {
                    bar.setState(Bar.BarState.CLIP);
                    follower.followPath(score2);
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.DTCLIP1);
                    wrist.setState(Wrist.wristState.DTCLIP);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy() || follower.isRobotStuck()) {
                    bar.setState(Bar.BarState.DTCLIP2);
                    setPathState(1401);
                }
                break;
            case 1401:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(score2ToWall, true);
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTime.getElapsedTimeSeconds() > 0.4) { //Todo needs heavy tuning,,,,
                    claw.setState(Claw.ClawState.SUPEROPEN);
                    bar.setState(Bar.BarState.DTWALL);
                    wrist.setState(Wrist.wristState.DTWALL);
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy() || follower.isRobotStuck()) { //Todo IDK!!!
                    claw.setState(Claw.ClawState.CLOSE);
                    setPathState(1601);
                }
                break;
            case 1601:
                if(pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.DTWALLSILLY);
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTime.getElapsedTimeSeconds() > 0.35) {
                    bar.setState(Bar.BarState.CLIP);
                    follower.followPath(score3);
                    setPathState(18);
                }
                break;
            case 18:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.DTCLIP1);
                    wrist.setState(Wrist.wristState.DTCLIP);
                    setPathState(19);
                }
                break;
            case 19:
                if (!follower.isBusy() || follower.isRobotStuck()) {
                    intakeWrist.setState(IntakeWrist.intakeWristState.IN);
                    bar.setState(Bar.BarState.DTCLIP2);
                    setPathState(1901);
                }
                break;
            case 1901:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(score3ToWall, true);
                    setPathState(20);
                }
                break;
            case 20:
                if (pathTime.getElapsedTimeSeconds() > 0.4) { //Todo needs heavy tuning,,,,
                    claw.setState(Claw.ClawState.SUPEROPEN);
                    bar.setState(Bar.BarState.DTWALL);
                    wrist.setState(Wrist.wristState.DTWALL);
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
        telemetry.addData("The way he ts chopped his ts", "\nis so tuff");
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
        telemetry.addData("claw state: ", claw.currentState);
        telemetry.addData("slides L/R ", slides.getLPos() + " / " + slides.getRPos());
        telemetry.update();
        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

}