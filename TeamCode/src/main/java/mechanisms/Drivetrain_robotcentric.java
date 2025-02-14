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

    public static boolean slowMode = false;

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
    public void Loop(Gamepad gp1) {
        if (slowMode) {
            drive.setTeleOpMovementVectors(-gp1.left_stick_y * 0.7, -gp1.left_stick_x * 0.7, -gp1.right_stick_x * 0.7, true);
        } else {
            drive.setTeleOpMovementVectors(-gp1.left_stick_y, -gp1.left_stick_x, -gp1.right_stick_x, true);
        }

        drive.update();
    }

    public void slowModeON(){
        slowMode = true;
    }

    public void slowModeOFF(){
        slowMode = false;
    }
}
