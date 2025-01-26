package testmechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "bwcsi", group = "testing")
public class bwcs extends LinearOpMode {
    MultipleTelemetry dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public BarTest bar = new BarTest();
    public WristTest wrist = new WristTest();
    public ClawTest claw = new ClawTest();
    public SlidesTest slides = new SlidesTest();
    public IntakeTest intake = new IntakeTest();
    public IntakeWristTest intakeWrist = new IntakeWristTest();

    @Override
    public void waitForStart() {
        super.waitForStart();
    }

    @Override
    public void runOpMode() {
        bar.init(hardwareMap);
        claw.init(hardwareMap);
        wrist.init(hardwareMap);
        slides.init(hardwareMap);
        intakeWrist.init(hardwareMap);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            bar.Loop();
            claw.Loop();
            wrist.Loop();
            slides.Loop();
            intakeWrist.Loop();
//            dashboardTelemetry.addData("Last commanded position:", "%.3f", bar.POS);
            dashboardTelemetry.update();
        }
    }
}
