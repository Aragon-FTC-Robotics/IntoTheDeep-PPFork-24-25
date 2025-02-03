import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import mechanisms.*;
@TeleOp(name="Color Telemetry")
public class ColorTelemetry extends LinearOpMode {
    Colorsensor color = new Colorsensor();
    @Override
    public void waitForStart() {
        super.waitForStart();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        color.init(hardwareMap);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            color.Loop();
            telemetry.addData("red? / yellow? / blue?", color.sensorIsRed() + " " + color.sensorIsYellow() + " " + color.sensorIsBlue());
            telemetry.addData("red/yellow/blue", color.colorHSV[0] + " " + color.colorHSV[1] + " " + color.colorHSV[2]);
            telemetry.addData("gain", Colorsensor.gain);
            telemetry.update();
        }
    }
}
