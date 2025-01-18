import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import mechanisms.Colorsensor;
import mechanisms.Extendo;
import mechanisms.Slides;

@TeleOp(name="RESER ENCODERS")
public class RESETENCODERS extends LinearOpMode {
    Extendo extendo = new Extendo();
    Slides slides = new Slides();
    @Override
    public void waitForStart() {
        super.waitForStart();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        slides.init(hardwareMap);
        extendo.init(hardwareMap);
        waitForStart();
        slides.DANGEROUS_RESET_ENCODERS();
        extendo.DANGEROUS_RESET_ENCODERS();
        while (opModeIsActive() && !isStopRequested()) {
            slides.Loop();
            extendo.Loop();
        }
    }
}
