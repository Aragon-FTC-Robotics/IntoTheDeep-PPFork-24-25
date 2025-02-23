package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.0029731058;
        ThreeWheelConstants.strafeTicksToInches = 0.0029417417;
        ThreeWheelConstants.turnTicksToInches = 0.0029588020;
        ThreeWheelConstants.leftY = 5.1;
        ThreeWheelConstants.rightY = -5.1;
        ThreeWheelConstants.strafeX = -5.25;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "rightRear";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "leftRear";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "rightFront";
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
//        ThreeWheelIMUConstants.forwardTicksToInches = 0.0029731058;
//        ThreeWheelIMUConstants.strafeTicksToInches = 0.0029417417;
//        ThreeWheelIMUConstants.turnTicksToInches = 0.0029588020;
//        ThreeWheelIMUConstants.leftY = 5.1;
//        ThreeWheelIMUConstants.rightY = -5.1;
//        ThreeWheelIMUConstants.strafeX = -5.25;
//        ThreeWheelIMUConstants.leftEncoder_HardwareMapName = "rightRear";
//        ThreeWheelIMUConstants.rightEncoder_HardwareMapName = "leftRear";
//        ThreeWheelIMUConstants.strafeEncoder_HardwareMapName = "rightFront";
//        ThreeWheelIMUConstants.leftEncoderDirection = Encoder.FORWARD;
//        ThreeWheelIMUConstants.rightEncoderDirection = Encoder.REVERSE;
//        ThreeWheelIMUConstants.strafeEncoderDirection = Encoder.REVERSE;
//        ThreeWheelIMUConstants.IMU_HardwareMapName = "imu";
//        ThreeWheelIMUConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);

    }
}




