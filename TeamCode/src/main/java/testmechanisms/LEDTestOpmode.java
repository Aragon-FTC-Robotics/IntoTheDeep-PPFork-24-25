package testmechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Led", group = "testing")
public class LEDTestOpmode extends LinearOpMode {
    MultipleTelemetry dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    LEDTest LED = new LEDTest();
    @Override
    public void waitForStart() {
        super.waitForStart();
    }
    @Override
    public void runOpMode() {
        LED.init(hardwareMap);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            LED.Loop();
            dashboardTelemetry.addData("Last commanded position:", "%.3f", LED.LED.getPosition());
            dashboardTelemetry.update();
        }
    }
}
