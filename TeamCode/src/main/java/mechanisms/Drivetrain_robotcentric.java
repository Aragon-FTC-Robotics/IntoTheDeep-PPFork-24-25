package mechanisms;

import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class Drivetrain_robotcentric {
    private Follower drive;
    private final Pose startPose = new Pose(0,0,0);
    private final Pose wallPose = new Pose(8.3, 24, Math.toRadians(180));
    private final Pose scorePose = new Pose(39, 72.6, Math.toRadians(180));
    public PathChain scorepath, returnpath;
    private double gamepadScalar = 1;
    public static boolean slowMode = false;
    public enum DriveMode {SPEC_SCORE, SPEC_RETURN, MANUAL};
    public DriveMode driveMode = DriveMode.MANUAL;
    /** This method is call once when init is played, it initializes the follower **/
    public void init(HardwareMap hm) {
        scorepath = drive.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(startPose), new Point(wallPose))))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        returnpath = drive.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(wallPose), new Point(startPose))))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        Constants.setConstants(FConstants.class,LConstants.class);
        drive = new Follower(hm);
        drive.setStartingPose(startPose);
    }

    /** This method is called once at the start of the OpMode. **/
    public void start() {
        drive.startTeleopDrive();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    public void Loop(Gamepad gp1, Gamepad gp2) {
        switch (driveMode) {
            case MANUAL:
                gamepadScalar = slowMode ? 0.5 : 1;
                if (Math.abs(gp1.left_stick_x) >= 0.05 || Math.abs(gp1.left_stick_y) >= 0.05 || Math.abs(gp1.right_stick_x) >= 0.05) {
                    drive.setTeleOpMovementVectors(-gp1.left_stick_y * gamepadScalar, -gp1.left_stick_x * gamepadScalar, -gp1.right_stick_x * gamepadScalar, true);
                } else {
                    drive.holdPoint(drive.getPose()); //unpushable robot
                }
                break;
            case SPEC_SCORE:
                drive.followPath(scorepath, false);
                if (!drive.isBusy()) {
                    driveMode = DriveMode.SPEC_RETURN;
                }
                break;
            case SPEC_RETURN:
                drive.followPath(returnpath, false);
                if (!drive.isBusy()) {
                    driveMode = DriveMode.MANUAL;
                }
                break;
        }
        drive.update();
    }
    public void startSpecScoring() {
        drive.setPose(wallPose);
        driveMode = DriveMode.SPEC_SCORE;
    }
    public void slowModeON(){
        slowMode = true;
    }

    public void slowModeOFF(){
        slowMode = false;
    }
}
