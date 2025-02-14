package mechanisms;

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

    /** This method is call once when init is played, it initializes the follower **/
    public void init(HardwareMap hm) {
        Constants.setConstants(FConstants.class,LConstants.class);
        drive = new Follower(hm);
        drive.setStartingPose(startPose);
    }

    /** This method is called once at the start of the OpMode. **/
    public void start() {
        drive.startTeleopDrive();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    public void loop(Gamepad gp1) {

        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: true
        */

        drive.setTeleOpMovementVectors(gp1.left_stick_y, gp1.left_stick_y, -gp1.right_stick_x, true);
        drive.update();
    }
}
