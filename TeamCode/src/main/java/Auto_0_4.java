import static mechanisms.Intake.intakeState.IN;
import static mechanisms.Intake.intakeState.STOP;
import static mechanisms.IntakeWrist.intakeWristState.OUT;
import static mechanisms.IntakeWrist.intakeWristState.SPIT;
import static mechanisms.IntakeWrist.intakeWristState.TRANSFER;

import android.util.Log;

import com.pedropathing.follower.*;
import com.pedropathing.localization.*;
import com.pedropathing.pathgen.*;
import com.pedropathing.util.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import mechanisms.*;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "0+4", group = "Auto")
public class Auto_0_4 extends OpMode {
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


    private final Pose STARTPOSE = new Pose(7.065,96.000, Math.toRadians(-90));
    private final Pose PRELOADPOSE = new Pose(-54.7453+72, 57.527+72, 5.5738);
    private final Pose INTAKE1POSE = new Pose(-55.734+72, 52.800+72, 6.10658);
    private final Pose INTAKE1POSEMISS = new Pose(-53.734+72, 52.800+72, 6.10658);
    private final Pose INTAKE2POSE = new Pose(-55.010+72, 58.218+72, 0.05);
    private final Pose INTAKE2POSEMISS = new Pose(-53.010+72, 58.218+72, 0.05);
    private final Pose INTAKE3POSE = new Pose(-50.395+72, 53.730+72, 0.4724);
    private final Pose BUCKETPOSE = new Pose(-54.7453+72, 57.527+72, 5.5738);
    private final Pose ASCENTPOSE = new Pose(9.396, 24.882, Math.toRadians(-10));
    private final Pose ASCENTCONTROL1 = new Pose(18.235, 70.235);

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
                        new BezierLine(new Point(INTAKE3POSE), new Point(BUCKETPOSE))
                )
                .setLinearHeadingInterpolation(INTAKE3POSE.getHeading(), BUCKETPOSE.getHeading())
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
                if (pathTime.getElapsedTimeSeconds()>0.30) {
                    intake.setState(IN);
                    intakeWrist.setState(IntakeWrist.intakeWristState.SUPERALMOSTOUT); //Maybe needs to be a different out position
                    setPathState(202);
                }
                break;
            case 202:
                if (pathTime.getElapsedTimeSeconds()>0.2){
                    intakeWrist.setState(IntakeWrist.intakeWristState.OUT);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTime.getElapsedTimeSeconds() > 2.25 || colorsensor.sensorIsYellow()) {
                    intake.setState(Intake.intakeState.STOP);


                    if(!colorsensor.sensorIsYellow()) { //FAIL
                        intakeWrist.setState(IntakeWrist.intakeWristState.SPIT);
                        Log.d("FAIL!!!!", "GO BACK TO SCHOOL");
                        follower.followPath(grab1_fix, true);
                        setPathState(301);
                    }
                    else { //Success
                        follower.followPath(score1, true); //Samp 1 -> bucket
                        intakeWrist.setState(TRANSFER);
                        extendo.setTargetPos(-100);
                        setPathState(4);
                    }
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
                if(pathTime.getElapsedTimeSeconds() > 2.5 || colorsensor.sensorIsYellow()) {
                    intake.setState(Intake.intakeState.STOP);

                    intakeWrist.setState(TRANSFER);
                    follower.followPath(score1_fix, true); //Samp 1 -> bucket
                    extendo.setTargetPos(-100);
                    setPathState(4);
                }
                break;
            case 4:
                if (pathTime.getElapsedTimeSeconds() > 1.5) {
                    claw.setState(Claw.ClawState.OPEN);
                    bar.setState(Bar.BarState.AUTOTRANSFER);
                    wrist.setState(Wrist.wristState.AUTOTRANSFER);
                    extendo.setTargetPos(Extendo.MIN);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
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
                if (pathTime.getElapsedTimeSeconds() > 0.6) {
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
                    setPathState(801);
                }
                break;
            case 801:
                if (pathTime.getElapsedTimeSeconds()>0.343) {
                    intake.setState(IN);
                    intakeWrist.setState(IntakeWrist.intakeWristState.OUT);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTime.getElapsedTimeSeconds() > 2.5 || colorsensor.sensorIsYellow()) {
                    intake.setState(Intake.intakeState.STOP);
                    if(!colorsensor.sensorIsYellow()) { //FAIL
                        Log.d("FAIL!!!!", "GO BACK TO SCHOOL");
                        intakeWrist.setState(IntakeWrist.intakeWristState.SPIT);
                        follower.followPath(grab2_fix, true);
                        setPathState(901);
                    }
                    else { //Success
                        follower.followPath(score2, true); //Samp 2 -> bucket
                        intakeWrist.setState(TRANSFER);
                        claw.setState(Claw.ClawState.CLOSE);
                        extendo.setTargetPos(-100);
                        setPathState(10);
                    }
                }
                break;
            case 901:
                if(pathTime.getElapsedTimeSeconds() > 0.5) {
                    intakeWrist.setState(IntakeWrist.intakeWristState.OUT);
                    intake.setState(IN);
                    setPathState(902);
                }
                break;
            case 902:
                if(pathTime.getElapsedTimeSeconds() > 2 || colorsensor.sensorIsYellow()) {
                    intake.setState(Intake.intakeState.STOP);
                    claw.setState(Claw.ClawState.CLOSE);
                    intakeWrist.setState(TRANSFER);
                    follower.followPath(score2_fix, true); //Samp 1 -> bucket
                    extendo.setTargetPos(-100);
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTime.getElapsedTimeSeconds() > 1.5) {
                    claw.setState(Claw.ClawState.OPEN);
                    bar.setState(Bar.BarState.AUTOTRANSFER);
                    wrist.setState(Wrist.wristState.AUTOTRANSFER);
                    extendo.setTargetPos(Extendo.MIN);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
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
                if (pathTime.getElapsedTimeSeconds() > 0.6) {
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
                    setPathState(1401);
                }
                break;
            case 1401:
                if (pathTime.getElapsedTimeSeconds() > 0.344) {
                    intake.setState(IN);
                    intakeWrist.setState(IntakeWrist.intakeWristState.OUT);
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTime.getElapsedTimeSeconds() > 2.5 || colorsensor.sensorIsYellow()) {
                    if (!colorsensor.sensorIsYellow()) {//FAIL
                        intakeWrist.setState(SPIT);
                        intake.setState(STOP);
                        setPathState(1501);
                    } else {
                        intake.setState(STOP);
                        extendo.setTargetPos(-100);
                        claw.setState(Claw.ClawState.CLOSE);
                        intakeWrist.setState(TRANSFER);
                        follower.followPath(score3, true); //Samp 3 -> bucket
                        setPathState(16);
                    }
                }
                break;
            case 1501:
                if(pathTime.getElapsedTimeSeconds()>1) {
                    intakeWrist.setState(OUT);
                    intake.setState(IN);
                    setPathState(1502);
                }
                break;
            case 1502:
                if (pathTime.getElapsedTimeSeconds() > 2 || colorsensor.sensorIsYellow()) {
                        intake.setState(STOP);
                        extendo.setTargetPos(-100);
                        claw.setState(Claw.ClawState.CLOSE);
                        intakeWrist.setState(TRANSFER);
                        follower.followPath(score3, true); //Samp 3 -> bucket
                        setPathState(16);
                    }
                break;

            case 16:
                if (pathTime.getElapsedTimeSeconds() > 1.5) {
                    bar.setState(Bar.BarState.AUTOTRANSFER);
                    wrist.setState(Wrist.wristState.AUTOTRANSFER);
                    claw.setState(Claw.ClawState.OPEN);
                    extendo.setTargetPos(Extendo.MIN);
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
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
                    follower.followPath(park, true); //bucket -> park
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
        extendo.Loop();
        intake.Loop();
        intakeWrist.Loop();
        slides.Loop();
        wrist.Loop();
        led.Loop();
        if(intake.getState()=="IN"){colorsensor.Loop();}
        telemetry.addData("path state", pathState);
        telemetry.addData("total time", totalTime);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
