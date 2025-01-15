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
        color.init(hardwareMap);
        while (opModeIsActive() && !isStopRequested()) {
            color.Loop();
            telemetry.addData("red? / yellow? / blue?", color.sensorIsRed() + " " + color.sensorIsYellow() + " " + color.sensorIsBlue());
            telemetry.addData("gain", Colorsensor.gain);
            telemetry.update();
        }
    }
}
