import static mechanisms.Intake.intakeState.IN;
import static mechanisms.Intake.intakeState.STOP;
import static mechanisms.IntakeWrist.intakeWristState.OUT;
import static mechanisms.IntakeWrist.intakeWristState.SPIT;
import static mechanisms.IntakeWrist.intakeWristState.SUPERALMOSTOUT;
import static mechanisms.IntakeWrist.intakeWristState.TRANSFER;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;

import mechanisms.Bar;
import mechanisms.Claw;
import mechanisms.Colorsensor;
import mechanisms.Extendo;
import mechanisms.Intake;
import mechanisms.IntakeWrist;
import mechanisms.LEDshenanigans;
import mechanisms.Slides;
import mechanisms.Wrist;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Disabled
@Config
@Autonomous(name = "🦷🗑️ NO COLOR bucket", group = "Auto")
public class Auto_0_4_Colorless extends OpMode {
    private Bar bar;
    private Claw claw;
    private Extendo extendo;
    private Intake intake;
    private IntakeWrist intakeWrist;
    private Slides slides;
    private Wrist wrist;
    private Colorsensor colorsensor;
    private LEDshenanigans led;

    private Follower follower;
    private Timer pathTime, totalTime;
    private int pathState = 0;
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;
    private Telemetry telemetryA;


    private static final Pose STARTPOSE = new Pose(7.065,96.000, Math.toRadians(-90));
    private static final Pose PRELOADPOSE = new Pose(-54.7453+72, 57.527+72, 5.5738);
    private static final Pose INTAKE1POSE = new Pose(-50.745+72, 35.325+72, 0.4734); //og: 6.264,  pos (16.62, 120.961)
    private static final Pose INTAKE1POSEMISS = new Pose(-50.245+72, 36.725+72, 0.4734); //og: 6.224
    private static final Pose INTAKE2POSE = new Pose(-52.0456+72, 47.7468+72, 0.2915); //og: 6.2103, pos (18.827, 131.935)
    private static final Pose INTAKE2POSEMISS = new Pose(-50.0456+72, 48.7468+72, 0.2915); //og: 6.1644 , pos (21.247, 132.2)
    private static final Pose INTAKE3POSE = new Pose(-49.442+72, 55.047+72, Math.toRadians(20)); //og: 0.296, (18.558, 129.047)
    private static final Pose BUCKETPOSE = new Pose(-54.7453+72, 57.527+72, 5.5788);
    private static final Pose FINALBUCKETPOSE = new Pose(-51.7453+72, 56.527+72, 5.5788);
    private static final Pose ASCENTPOSE = new Pose(-16, 24.882, Math.toRadians(0));
    private static final Pose ASCENTCONTROL1 = new Pose(18.235, 70.235);

    private Path scorePreload, park;
    private PathChain grab1, grab2, grab3, score1, score1_fix, score2, score3, grab1_fix, grab2_fix, score2_fix;

    private void buildPaths() {
        //Start -> Preload bucket pos
        scorePreload = new Path(new BezierLine(new Point(STARTPOSE), new Point(PRELOADPOSE)));
        scorePreload.setLinearHeadingInterpolation(STARTPOSE.getHeading(), PRELOADPOSE.getHeading());

        //Preload bucket pos -> Intake 1
        grab1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Point(PRELOADPOSE), new Point(INTAKE1POSE))
                )
                .setLinearHeadingInterpolation(PRELOADPOSE.getHeading(), INTAKE1POSE.getHeading())
                .build();
        grab1_fix = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Point(INTAKE1POSE), new Point(INTAKE1POSEMISS))
                )
                .setLinearHeadingInterpolation(INTAKE1POSE.getHeading(), INTAKE1POSEMISS.getHeading())
                .build();
        grab2_fix = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Point(INTAKE2POSE), new Point(INTAKE2POSEMISS))
                )
                .setLinearHeadingInterpolation(INTAKE2POSE.getHeading(), INTAKE2POSEMISS.getHeading())
                .build();
        score2_fix  = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Point(INTAKE2POSEMISS), new Point(BUCKETPOSE))
                )
                .setLinearHeadingInterpolation(INTAKE2POSEMISS.getHeading(), BUCKETPOSE.getHeading())
                .build();

        //Intake 1 -> bucket
        score1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Point(INTAKE1POSE), new Point(BUCKETPOSE))
                )
                .setLinearHeadingInterpolation(INTAKE1POSE.getHeading(), BUCKETPOSE.getHeading())
                .build();
        score1_fix = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Point(INTAKE1POSEMISS), new Point(BUCKETPOSE))
                )
                .setLinearHeadingInterpolation(INTAKE1POSEMISS.getHeading(), BUCKETPOSE.getHeading())
                .build();

        //Bucket -> intake 2
        grab2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Point(BUCKETPOSE), new Point(INTAKE2POSE))
                )
                .setLinearHeadingInterpolation(BUCKETPOSE.getHeading(), INTAKE2POSE.getHeading())
                .build();

        //intake 2 -> bucket
        score2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Point(INTAKE2POSE), new Point(BUCKETPOSE))
                )
                .setLinearHeadingInterpolation(INTAKE2POSE.getHeading(), BUCKETPOSE.getHeading())
                .build();

        //bucket -> intake 3
        grab3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Point(BUCKETPOSE), new Point(INTAKE3POSE))
                )
                .setLinearHeadingInterpolation(BUCKETPOSE.getHeading(), INTAKE3POSE.getHeading())
                .build();

        //intake 3 -> bucket
        score3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Point(INTAKE3POSE), new Point(FINALBUCKETPOSE))
                )
                .setLinearHeadingInterpolation(INTAKE3POSE.getHeading(), FINALBUCKETPOSE.getHeading())
                .build();

        //Bucket -> ascent (bezier curve)
        park = new Path(new BezierCurve(
                new Point(BUCKETPOSE),
                new Point(ASCENTCONTROL1),
                new Point(ASCENTPOSE)
            )
        );
    }
    private void updatePaths() {
        switch (pathState) {
            case 0:
                slides.setTargetPos(slides.HIGH);
                bar.setState(Bar.BarState.BUCKET);
                wrist.setState(Wrist.wristState.BUCKET);
                claw.setState(Claw.ClawState.CLOSE); //holding sample the other way
                extendo.setTargetPos(-100); // avoid warping
                follower.followPath(scorePreload); //Start -> preload score
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && pathTime.getElapsedTimeSeconds() > 0.45) {
                    claw.setState(Claw.ClawState.OPEN);
                    setPathState(101);
                }
                break;
            case 101:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(grab1, true); //preload score -> samp1
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    slides.setTargetPos(slides.GROUND);
                    bar.setState(Bar.BarState.NEUTRAL);
                    wrist.setState(Wrist.wristState.NEUTRAL);
                    claw.setState(Claw.ClawState.CLOSE);
                    extendo.setTargetPos(Extendo.MAX);
                    setPathState(201);
                }
                break;
            case 201:
                if (pathTime.getElapsedTimeSeconds()>0.15) {
                    intake.setState(IN);
                    intakeWrist.setState(IntakeWrist.intakeWristState.SUPERALMOSTOUT); //Maybe needs to be a different out position
                    setPathState(202);
                }
                break;
            case 202:
                if (extendo.getPos()>1900){
                    intakeWrist.setState(IntakeWrist.intakeWristState.OUT);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTime.getElapsedTimeSeconds() > 2) {
                    intake.setState(Intake.intakeState.STOP);



                        intakeWrist.setState(IntakeWrist.intakeWristState.SPIT);
                        Log.d("FAIL!!!!", "GO BACK TO SCHOOL");
                        follower.followPath(grab1_fix, true);
                        setPathState(301);
                }
                break;
            case 301:
                if(pathTime.getElapsedTimeSeconds() > 0.5) {
                    intakeWrist.setState(IntakeWrist.intakeWristState.OUT);
                    intake.setState(IN);
                    setPathState(302);
                }
                break;
            case 302:
                if(pathTime.getElapsedTimeSeconds() > 2) {
                    intake.setState(Intake.intakeState.STOP);

                    intakeWrist.setState(TRANSFER);
                    follower.followPath(score1_fix, true); //Samp 1 -> bucket
                    extendo.setTargetPos(-100);
                    setPathState(4);
                }
                break;
            case 4:
                if (pathTime.getElapsedTimeSeconds() > 0.2) {
                    extendo.setTargetPos(Extendo.MIN);
                    wrist.setState(Wrist.wristState.AUTOTRANSFER);
                    setPathState(40001);
                }
                break;
            case 40001:
                if (pathTime.getElapsedTimeSeconds() > 0.75){
                    claw.setState(Claw.ClawState.OPEN);
                    setPathState(40002);
                }
            case 40002:
                if (pathTime.getElapsedTimeSeconds() > 0.75) {
                    bar.setState(Bar.BarState.AUTOTRANSFER);
                    setPathState(5);
                }
                break;
            case 401:
                if (!follower.isBusy()) {
                    setPathState(701);
                }
                 break;
            case 5:
                if (pathTime.getElapsedTimeSeconds() > 0.75) {
                    claw.setState(Claw.ClawState.CLOSE);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    extendo.setTargetPos(-100);
                    slides.setTargetPos(Slides.HIGH);
                    setPathState(6001);
                }
                break;
            case 6001:
                if (pathTime.getElapsedTimeSeconds() > 1){
                    bar.setState(Bar.BarState.BUCKET);
                    wrist.setState(Wrist.wristState.BUCKET);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTime.getElapsedTimeSeconds() > 0.8) {
                    intakeWrist.setState(IntakeWrist.intakeWristState.IN);
                    claw.setState(Claw.ClawState.OPEN);
                    setPathState(701);
                }
                break;
            case 701:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(grab2, true); //bucket -> samp2
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    slides.setTargetPos(slides.GROUND);
                    bar.setState(Bar.BarState.NEUTRAL);
                    wrist.setState(Wrist.wristState.NEUTRAL);
                    claw.setState(Claw.ClawState.CLOSE);
                    extendo.setTargetPos(Extendo.MAX-0);
                    intakeWrist.setState(SUPERALMOSTOUT);
                    intake.setState(IN);
                    setPathState(801);
                }
                break;
            case 801:
                if (extendo.getPos()>1800) {
                    intakeWrist.setState(IntakeWrist.intakeWristState.OUT);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTime.getElapsedTimeSeconds() > 2) {
                    intake.setState(Intake.intakeState.STOP);
                    Log.d("FAIL!!!!", "GO BACK TO SCHOOL");
                    intakeWrist.setState(IntakeWrist.intakeWristState.SPIT);
                    follower.followPath(grab2_fix, true);
                    setPathState(901);
                }
                break;
            case 901:
                if(pathTime.getElapsedTimeSeconds() > 0.5) {
                    intakeWrist.setState(IntakeWrist.intakeWristState.SUPEROUT);
                    intake.setState(IN);
                    setPathState(902);
                }
                break;
            case 902:
                if(pathTime.getElapsedTimeSeconds() > 2) {
                    intake.setState(Intake.intakeState.STOP);
                    claw.setState(Claw.ClawState.CLOSE);
                    intakeWrist.setState(TRANSFER);
                    follower.followPath(score2_fix, true); //Samp 1 -> bucket
                    extendo.setTargetPos(-100);
                    setPathState(10);
                }
                break;
            case 9001:
                if(follower.isBusy()) {
                    setPathState(1301);
                }
            case 10:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    claw.setState(Claw.ClawState.OPEN);
                    wrist.setState(Wrist.wristState.AUTOTRANSFER);
                    setPathState(10001);
                }
                break;
            case 10001:
                if (pathTime.getElapsedTimeSeconds() > 1.0) {
                    bar.setState(Bar.BarState.AUTOTRANSFER);
                    extendo.setTargetPos(Extendo.MIN);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTime.getElapsedTimeSeconds() > 0.75) {
                    claw.setState(Claw.ClawState.CLOSE);
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    extendo.setTargetPos(-100);
                    slides.setTargetPos(Slides.HIGH);
                    setPathState(1201);
                }
                break;
            case 1201:
                if (pathTime.getElapsedTimeSeconds() > 1){
                    bar.setState(Bar.BarState.BUCKET);
                    wrist.setState(Wrist.wristState.BUCKET);
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTime.getElapsedTimeSeconds() > 0.7) {
                    intakeWrist.setState(IntakeWrist.intakeWristState.IN);
                    claw.setState(Claw.ClawState.OPEN);
                    setPathState(1301);
                }
                break;
            case 1301:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(grab3, true); //bucket -> samp3
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    slides.setTargetPos(slides.GROUND);
                    bar.setState(Bar.BarState.NEUTRAL);
                    wrist.setState(Wrist.wristState.NEUTRAL);
                    extendo.setTargetPos(Extendo.MAX-00);
                    intakeWrist.setState(SUPERALMOSTOUT);
                    intake.setState(IN);
                    setPathState(1401);
                }
                break;
            case 1401:
                if (extendo.getPos()>1800) {
                    intakeWrist.setState(OUT);
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTime.getElapsedTimeSeconds() > 2) {
                    intakeWrist.setState(SPIT);
                    intake.setState(STOP);
                    setPathState(1501);
                }
                break;
            case 15001:
                if(!follower.isBusy()) {
                    setPathState(1901);
                }
            case 1501:
                if(pathTime.getElapsedTimeSeconds()>0.5) {
                    intakeWrist.setState(OUT);
                    intake.setState(IN);
                    setPathState(1502);
                }
                break;
            case 1502:
                if (pathTime.getElapsedTimeSeconds() > 2) {
                        intake.setState(STOP);
                        extendo.setTargetPos(-100);
                        claw.setState(Claw.ClawState.CLOSE);
                        intakeWrist.setState(TRANSFER);
                        follower.followPath(score3, true); //Samp 3 -> bucket
                        setPathState(16);
                    }
                break;
            case 15002:
                if(!follower.isBusy()) {
                    setPathState(1901);
                }
                break;

            case 16:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    wrist.setState(Wrist.wristState.AUTOTRANSFER);
                    claw.setState(Claw.ClawState.OPEN);
                    setPathState(160001);
                }
                break;
            case 160001:
                if (pathTime.getElapsedTimeSeconds() > 1.0) {
                    bar.setState(Bar.BarState.AUTOTRANSFER);
                    extendo.setTargetPos(Extendo.MIN);
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTime.getElapsedTimeSeconds() > 0.75) {
                    claw.setState(Claw.ClawState.CLOSE);
                    setPathState(18);
                }
                break;
            case 18:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    extendo.setTargetPos(-100);
                    slides.setTargetPos(Slides.HIGH);
                    setPathState(1801);
                }
                break;
            case 1801:
                if (pathTime.getElapsedTimeSeconds() > 1){
                    bar.setState(Bar.BarState.BUCKET);
                    wrist.setState(Wrist.wristState.BUCKET);
                    setPathState(19);
                }
                break;
            case 19:
                if (pathTime.getElapsedTimeSeconds() > 0.6) {
                    intakeWrist.setState(IntakeWrist.intakeWristState.IN);
                    claw.setState(Claw.ClawState.OPEN);
                    setPathState(1901);
                }
                break;
            case 1901:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
//                    follower.followPath(park, true); //bucket -> park
                    setPathState(20);
                }
                break;
            case 20:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    slides.setTargetPos(Slides.GROUND); //Hopefully no lvl 4
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
        pathTime = new Timer();
        totalTime = new Timer();
        totalTime.resetTimer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(STARTPOSE);
        follower.setMaxPower(1);
        poseUpdater = new PoseUpdater(hardwareMap);
        poseUpdater.setStartingPose(new Pose(STARTPOSE.getX(), STARTPOSE.getY(), STARTPOSE.getHeading()));

        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
        buildPaths();

        bar = new Bar();
        claw = new Claw();
        extendo = new Extendo();
        intake = new Intake();
        intakeWrist = new IntakeWrist();
        slides = new Slides();
        wrist = new Wrist();
        colorsensor = new Colorsensor();
        led = new LEDshenanigans();

        bar.init(hardwareMap);
        claw.init(hardwareMap);
        extendo.init(hardwareMap);
        intake.init(hardwareMap);
        intakeWrist.init(hardwareMap);
        slides.init(hardwareMap);
        wrist.init(hardwareMap);
        colorsensor.init(hardwareMap);
        led.init(hardwareMap);
        slides.DANGEROUS_RESET_ENCODERS();
        extendo.DANGEROUS_RESET_ENCODERS();
        claw.setState(Claw.ClawState.CLOSE);
        bar.setState(Bar.BarState.WALL);
        wrist.setState(Wrist.wristState.WALL);
        claw.Loop(); //Runs loop once to move to position
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
        extendo.Loop(13);
        intake.Loop();
        intakeWrist.Loop();
        slides.Loop(13);
        wrist.Loop();
        led.Loop();
        if(intake.getState()=="IN"){colorsensor.Loop();}
        telemetry.addData("path state", pathState);
        telemetry.addData("total time", totalTime);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        if(totalTime.getElapsedTimeSeconds() > 28) {
            setPathState(-1);
            bar.setState(Bar.BarState.PARK);
            wrist.setState(Wrist.wristState.PARK);
        }
        telemetry.update();
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(new Pose(follower.getPose().getX(),follower.getPose().getY(),follower.getPose().getHeading()), "blue");
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

    }
}
