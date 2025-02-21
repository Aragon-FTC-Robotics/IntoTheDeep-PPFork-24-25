import static mechanisms.Intake.intakeState.IN;
import static mechanisms.Intake.intakeState.STOP;
import static mechanisms.IntakeWrist.intakeWristState.OUT;
import static mechanisms.IntakeWrist.intakeWristState.SPIT;
import static mechanisms.IntakeWrist.intakeWristState.SUPERALMOSTOUT;
import static mechanisms.IntakeWrist.intakeWristState.SUPEROUT;
import static mechanisms.IntakeWrist.intakeWristState.TRANSFER;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Config
@Autonomous(name = "New bucket auto", group = "Auto")
public class Auto_no_extend extends OpMode {
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
    private PathChain scorePreload, prepare1, scoot1, score1, prepare2, scoot2, score2, prepare3, scoot3, score3, park;
    private void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(7.065, 108.000, Point.CARTESIAN),
                                new Point(15.000, 126.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45))
                .build();

        prepare1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(15, 126.500, Point.CARTESIAN),
                                new Point(24, 113, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(20))
                .build();

        scoot1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(24, 113, Point.CARTESIAN),
                                new Point(50, 123, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(20))
                .build();

        score1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(50.000, 123, Point.CARTESIAN),
                                new Point(33, 108, Point.CARTESIAN),
                                new Point(15.000, 126.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(-45))
                .build();

        prepare2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(15.000, 126.000, Point.CARTESIAN),
                                new Point(24.000, 123, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(20))
                .build();

        scoot2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(24.000, 123, Point.CARTESIAN),
                                new Point(50.000, 133, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(20))
                .build();

        score2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(50.000, 133, Point.CARTESIAN),
                                new Point(15.000, 126.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(-45))
                .build();

        prepare3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(15.000, 126.000, Point.CARTESIAN),
                                new Point(36.000, 130.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(50))
                .build();

        scoot3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(36.000, 130.000, Point.CARTESIAN),
                                new Point(44.000, 135, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(50))
                .build();

       score3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(44.000, 135, Point.CARTESIAN),
                                new Point(15.000, 126.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(-45))
                .build();

        park = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(15.000, 126.000, Point.CARTESIAN),
                                new Point(70.000, 130.000, Point.CARTESIAN),
                                new Point(64.000, 93, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90))
                .build();
    }
    private void updatePaths() {
        switch (pathState) {
            case 0:
                slides.setTargetPos(slides.HIGH);
                claw.setState(Claw.ClawState.CLOSE); //holding sample the other way
                extendo.setTargetPos(-100); // avoid warping
                follower.followPath(scorePreload); //Start -> preload score
                setPathState(101);
                break;
            case 101:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.BUCKET);
                    wrist.setState(Wrist.wristState.BUCKET);
                    setPathState(1);
                }
                break;
            case 1:
                if (!follower.isBusy() && pathTime.getElapsedTimeSeconds() > 1) {
                    claw.setState(Claw.ClawState.OPEN);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    slides.setTargetPos(Slides.GROUND);
                    bar.setState(Bar.BarState.NEUTRAL);
                    wrist.setState(Wrist.wristState.AUTOTRANSFER);
                    follower.followPath(prepare1, true); //preload score -> samp1
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    intakeWrist.setState(SUPEROUT);
                    intake.setState(IN);
                    follower.followPath(scoot1, 0.3, false);
                    setPathState(4);
                }
                break;
            case 4:
                if (colorsensor.sensorIsYellow() || !follower.isBusy()) {
                    intake.setState(STOP);
                    intakeWrist.setState(IntakeWrist.intakeWristState.TRANSFER);
                    follower.followPath(score1);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.AUTOTRANSFER);
                    wrist.setState(Wrist.wristState.AUTOTRANSFER);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTime.getElapsedTimeSeconds() > 0.8) {
                    claw.setState(Claw.ClawState.CLOSE);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTime.getElapsedTimeSeconds() > 0.75) {
                    intakeWrist.setState(IntakeWrist.intakeWristState.IN);
                    slides.setTargetPos(Slides.HIGH);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTime.getElapsedTimeSeconds() > 1) {
                    bar.setState(Bar.BarState.BUCKET);
                    wrist.setState(Wrist.wristState.BUCKET);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    claw.setState(Claw.ClawState.OPEN);
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    slides.setTargetPos(Slides.GROUND);
                    bar.setState(Bar.BarState.NEUTRAL);
                    wrist.setState(Wrist.wristState.AUTOTRANSFER);
                    follower.followPath(prepare2, true); //preload score -> samp1
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    intakeWrist.setState(SUPEROUT);
                    intake.setState(IN);
                    follower.followPath(scoot2, 0.3, false);
                    setPathState(12);
                }
                break;
            case 12:
                if (colorsensor.sensorIsYellow() || !follower.isBusy()) {
                    intake.setState(STOP);
                    intakeWrist.setState(IntakeWrist.intakeWristState.TRANSFER);
                    follower.followPath(score2);
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.AUTOTRANSFER);
                    wrist.setState(Wrist.wristState.AUTOTRANSFER);
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTime.getElapsedTimeSeconds() > 0.8) {
                    claw.setState(Claw.ClawState.CLOSE);
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    intakeWrist.setState(IntakeWrist.intakeWristState.IN);
                    slides.setTargetPos(Slides.HIGH);
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTime.getElapsedTimeSeconds() > 1) {
                    bar.setState(Bar.BarState.BUCKET);
                    wrist.setState(Wrist.wristState.BUCKET);
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    claw.setState(Claw.ClawState.OPEN);
                    setPathState(18);
                }
                break;
            case 18:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    slides.setTargetPos(Slides.GROUND);
                    bar.setState(Bar.BarState.NEUTRAL);
                    wrist.setState(Wrist.wristState.AUTOTRANSFER);
                    intakeWrist.setState(SUPEROUT);
                    follower.followPath(prepare3, true); //preload score -> samp1
                    setPathState(19);
                }
                break;
            case 19:
                if (!follower.isBusy()) {
                    intake.setState(IN);
                    follower.followPath(scoot3, 0.4, false);
                    setPathState(20);
                }
                break;
            case 20:
                if (!follower.isBusy() || follower.isRobotStuck() || pathTime.getElapsedTimeSeconds() > 2.3) {
                    intake.setState(STOP);
                    intakeWrist.setState(IntakeWrist.intakeWristState.TRANSFER);
                    follower.followPath(score3);
                    setPathState(21);
                }
                break;
            case 21:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.AUTOTRANSFER);
                    wrist.setState(Wrist.wristState.AUTOTRANSFER);
                    setPathState(22);
                }
                break;
            case 22:
                if (pathTime.getElapsedTimeSeconds() > 0.8) {
                    claw.setState(Claw.ClawState.CLOSE);
                    setPathState(23);
                }
                break;
            case 23:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    intakeWrist.setState(IntakeWrist.intakeWristState.IN);
                    slides.setTargetPos(Slides.HIGH);
                    setPathState(24);
                }
                break;
            case 24:
                if (pathTime.getElapsedTimeSeconds() > 1) {
                    bar.setState(Bar.BarState.BUCKET);
                    wrist.setState(Wrist.wristState.BUCKET);
                    setPathState(25);
                }
                break;
            case 25:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    claw.setState(Claw.ClawState.OPEN);
                    setPathState(26);
                }
                break;
            case 26:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.BUCKET);
                    wrist.setState(Wrist.wristState.BUCKET);
                    follower.followPath(park , 0.8, true);
                    setPathState(27);
                }
                break;
            case 27:
                if (pathTime.getElapsedTimeSeconds() > 1) {
                    slides.setTargetPos(Slides.GROUND);
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
        follower.setStartingPose(new Pose(7.065, 108.000, Math.toRadians(-90)));
        follower.setMaxPower(0.6);
        poseUpdater = new PoseUpdater(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
//        if(totalTime.getElapsedTimeSeconds() > 28) {
//            setPathState(-1);
//            bar.setState(Bar.BarState.PARK);
//            wrist.setState(Wrist.wristState.PARK);
//        }
        telemetry.update();
        follower.telemetryDebug(telemetry);

    }
}
