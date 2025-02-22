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

@Autonomous(name = "Clip intake auto", group = "Auto")
public class Auto_clipintake extends OpMode {
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
    int scoredspecimens = 0;
    private PathChain prepare1, spit1, prepare2, spit2, prepare3, spit3, spit3towall, score1, return1, score2, return2, score3, return3, score4, return4, score5, return5;
    private void buildPaths() {
        prepare1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(6.750, 43.000, Point.CARTESIAN),
                                new Point(41.000, 18.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))
                .build();
        spit1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(41.000, 18.000, Point.CARTESIAN),
                                new Point(14.000, 23.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-135))
                .build();
        prepare2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(14.000, 23.000, Point.CARTESIAN),
                                new Point(37.000, 16.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(-30))
                .build();
        spit2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(37.000, 16.000, Point.CARTESIAN),
                                new Point(14.000, 13.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-135))
                .build();
        prepare3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(14.000, 13.000, Point.CARTESIAN),
                                new Point(37.000, 20.000, Point.CARTESIAN),
                                new Point(47, 6.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(-80))
                .build();
        spit3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Point(47, 6.000, Point.CARTESIAN),
                                new Point(14.000, 30.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-80), Math.toRadians(180))
                .build();
        spit3towall = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(14.000, 30.000, Point.CARTESIAN),
                                new Point(6.5, 30.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        score1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(6, 30.000, Point.CARTESIAN),
                                new Point(37.000, 70.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        return1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(40, 70, Point.CARTESIAN),
                                new Point(6, 30, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    private void updatePaths() {
        switch (pathState) {
            case 0:
                extendo.setTargetPos(-100);
                intakeWrist.setState(IntakeWrist.intakeWristState.SUPEROUT);
                intake.setState(Intake.intakeState.IN);
                bar.setState(Bar.BarState.NEUTRAL);
                wrist.setState(Wrist.wristState.WALL);
                claw.setState(Claw.ClawState.OPEN);
                follower.followPath(prepare1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    intake.setState(Intake.intakeState.STOP);
                    intakeWrist.setState(IntakeWrist.intakeWristState.SPIT);
                    follower.followPath(spit1);
                    setPathState(2);
                }
                break;
            case 2:
                if (follower.atParametricEnd() || !follower.isBusy()) {
                    intake.setState(Intake.intakeState.OUT);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTime.getElapsedTimeSeconds() > 1) {
                    intakeWrist.setState(IntakeWrist.intakeWristState.SUPEROUT);
                    intake.setState(Intake.intakeState.IN);
                    follower.followPath(prepare2);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    intake.setState(Intake.intakeState.STOP);
                    intakeWrist.setState(IntakeWrist.intakeWristState.SPIT);
                    follower.followPath(spit2);
                    setPathState(5);
                }
                break;
            case 5:
                if (follower.atParametricEnd() || !follower.isBusy()) {
                    intake.setState(Intake.intakeState.OUT);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTime.getElapsedTimeSeconds() > 1) {
                    intakeWrist.setState(IntakeWrist.intakeWristState.SUPEROUT);
                    intake.setState(Intake.intakeState.IN);
                    follower.followPath(prepare3);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() || follower.isRobotStuck()) {
                    claw.setState(Claw.ClawState.OPEN);
                    intakeWrist.setState(IntakeWrist.intakeWristState.SPIT);
                    follower.followPath(spit3);
                    setPathState(8);
                }
                break;
            case 8:
                if (follower.getCurrentTValue() > 0.92) { //TODO: Find good t value for spitting
                    intake.setState(Intake.intakeState.OUT);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTime.getElapsedTimeSeconds() > 1) {
                    intake.setState(Intake.intakeState.STOP);
                    intakeWrist.setState(IntakeWrist.intakeWristState.IN);
                    setPathState(901);
                }
                break;
            case 901:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.WALL);
                    setPathState(902);
                }
                break;
            case 902:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(spit3towall, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (scoredspecimens == 5) {setPathState(-1);}
                if (!follower.isBusy()) {
                    claw.setState(Claw.ClawState.CLOSE);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTime.getElapsedTimeSeconds() > 0.8) { //TODO: Change how long claw takes to close
                    bar.setState(Bar.BarState.DTCLIP1);
                    wrist.setState(Wrist.wristState.DTCLIP);
                    follower.followPath(score1, 1, false);
                    setPathState(12);
                }
                break;
            case 12:
                if (follower.atParametricEnd() || !follower.isBusy()) {
                    bar.setState(Bar.BarState.DTCLIP2);
                    follower.followPath(return1, 0.8, true);
                    setPathState(13);
                }
                break;
            case 13:
                if (follower.getCurrentTValue() > 0.06) {
                    claw.setState(Claw.ClawState.OPEN);
                    scoredspecimens++;
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTime.getElapsedTimeSeconds() > 0.5) {
                    bar.setState(Bar.BarState.WALL);
                    wrist.setState(Wrist.wristState.WALL);
                    setPathState(10);
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
        follower.setStartingPose(new Pose(6.750, 43.000, Math.toRadians(0)));
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

        follower.update();
    }
    @Override
    public void start() {
        follower.drawOnDashBoard();
        led.setPosition(0.522);
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
        follower.telemetryDebug(telemetry);
        telemetry.update();
        loopTime.resetTimer();
    }

}
