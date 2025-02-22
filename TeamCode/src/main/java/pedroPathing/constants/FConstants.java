package pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.THREE_WHEEL;

        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftRear";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightRear";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 13.51705;

        FollowerConstants.xMovement = 76.10065606800222;
        FollowerConstants.yMovement = 61.94895697437406;

        FollowerConstants.forwardZeroPowerAcceleration = -38.223135130196724;
        FollowerConstants.lateralZeroPowerAcceleration = -70.9109666196431;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.1,0,0,0);
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.3,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.1015,0.01);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.010,0,0.0012,0,0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.02,0,0.000079,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 5.5;
        FollowerConstants.centripetalScaling = 0.00030;

        FollowerConstants.pathEndTimeoutConstraint = 500; //Up this number?
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.05; //Up this number?
        FollowerConstants.pathEndTranslationalConstraint = 0.05;
        FollowerConstants.pathEndHeadingConstraint = 0.007;

        FollowerConstants.useBrakeModeInTeleOp = true;
    }
}
