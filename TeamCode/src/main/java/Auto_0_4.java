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


    private Follower follower;
    private Timer pathTime, totalTime;
    private int pathState = 0;


    private final Pose STARTPOSE = new Pose(7.065,96.000, Math.toRadians(-90));
    private final Pose PRELOADPOSE = new Pose(-54.7453+72, 57.527+72, 5.5738);
    private final Pose INTAKE1POSE = new Pose(-55.934+72, 52.800+72, 6.10658);
    private final Pose INTAKE2POSE = new Pose(-55.010+72, 58.218+72, 0.05);
    private final Pose INTAKE3POSE = new Pose(-50.395+72, 53.730+72, 0.4854);
    private final Pose BUCKETPOSE = new Pose(-54.7453+72, 57.527+72, 5.5738);
    private final Pose ASCENTPOSE = new Pose(-7.786+72, 20.307+72, Math.toRadians(-90));
    private final Pose ASCENTCONTROL1 = new Pose(84, 129);

    private Path scorePreload, park;
    private PathChain grab1, grab2, grab3, score1, score2, score3;

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

        //Intake 1 -> bucket
        score1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Point(INTAKE1POSE), new Point(BUCKETPOSE))
                )
                .setLinearHeadingInterpolation(INTAKE1POSE.getHeading(), BUCKETPOSE.getHeading())
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
                    extendo.setTargetPos(Extendo.MAX-100);
                    setPathState(201);
                }
                break;
            case 201:
                if (pathTime.getElapsedTimeSeconds()>0.3) {
                    intake.setState(Intake.intakeState.IN);
                    intakeWrist.setState(IntakeWrist.intakeWristState.OUT); //Maybe needs to be a different out position
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTime.getElapsedTimeSeconds() > 3) {
                    intake.setState(Intake.intakeState.STOP);
                    extendo.setTargetPos(Extendo.MIN);
                    intakeWrist.setState(IntakeWrist.intakeWristState.TRANSFER);
                    follower.followPath(score1, true); //Samp 1 -> bucket
                    setPathState(4);
                }
                break;
            case 4:
                if (pathTime.getElapsedTimeSeconds() > 1.5) {
                    bar.setState(Bar.BarState.TRANSFER);
                    wrist.setState(Wrist.wristState.TRANSFER);
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
                    slides.setTargetPos(Slides.HIGH);
                    setPathState(6001);
                }
                break;
            case 6001:
                if (pathTime.getElapsedTimeSeconds() > 0.6){
                    bar.setState(Bar.BarState.BUCKET);
                    wrist.setState(Wrist.wristState.BUCKET);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTime.getElapsedTimeSeconds() > 2) {
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
                    extendo.setTargetPos(Extendo.MAX-100);
                    setPathState(801);
                }
                break;
            case 801:
                if (pathTime.getElapsedTimeSeconds()>0.343) {
                    intake.setState(Intake.intakeState.IN);
                    intakeWrist.setState(IntakeWrist.intakeWristState.OUT);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTime.getElapsedTimeSeconds() > 3) {
                    intake.setState(Intake.intakeState.STOP);
                    extendo.setTargetPos(Extendo.MIN);
                    intakeWrist.setState(IntakeWrist.intakeWristState.TRANSFER);
                    follower.followPath(score2, true); //Samp 2 -> bucket
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTime.getElapsedTimeSeconds() > 1.5) {
                    bar.setState(Bar.BarState.TRANSFER);
                    wrist.setState(Wrist.wristState.TRANSFER);
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
                    slides.setTargetPos(Slides.HIGH);
                    setPathState(1201);
                }
                break;
            case 1201:
                if (pathTime.getElapsedTimeSeconds() > 0.6){
                    bar.setState(Bar.BarState.BUCKET);
                    wrist.setState(Wrist.wristState.BUCKET);
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTime.getElapsedTimeSeconds() > 2) {
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
                    extendo.setTargetPos(Extendo.MAX-100);
                    setPathState(1401);
                }
                break;
            case 1401:
                if (pathTime.getElapsedTimeSeconds() > 0.344) {
                    intake.setState(Intake.intakeState.IN);
                    intakeWrist.setState(IntakeWrist.intakeWristState.OUT);
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTime.getElapsedTimeSeconds() > 3) {
                    intake.setState(Intake.intakeState.STOP);
                    extendo.setTargetPos(Extendo.MIN);
                    intakeWrist.setState(IntakeWrist.intakeWristState.TRANSFER);
                    follower.followPath(score3, true); //Samp 3 -> bucket
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTime.getElapsedTimeSeconds() > 1.5) {
                    bar.setState(Bar.BarState.TRANSFER);
                    wrist.setState(Wrist.wristState.TRANSFER);
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
                    slides.setTargetPos(Slides.HIGH);
                    setPathState(1801);
                }
                break;
            case 1801:
                if (pathTime.getElapsedTimeSeconds() > 0.6){
                    bar.setState(Bar.BarState.BUCKET);
                    wrist.setState(Wrist.wristState.BUCKET);
                    setPathState(19);
                }
                break;
            case 19:
                if (pathTime.getElapsedTimeSeconds() > 2) {
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
                    bar.setState(Bar.BarState.CLIP);
                    wrist.setState(Wrist.wristState.CLIP);
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

        bar.init(hardwareMap);
        claw.init(hardwareMap);
        extendo.init(hardwareMap);
        intake.init(hardwareMap);
        intakeWrist.init(hardwareMap);
        slides.init(hardwareMap);
        wrist.init(hardwareMap);

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
        telemetry.addData("path state", pathState);
        telemetry.addData("total time", totalTime);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
