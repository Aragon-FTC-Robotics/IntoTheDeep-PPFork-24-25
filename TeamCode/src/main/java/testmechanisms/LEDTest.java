package testmechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class LEDTest {
    Servo LED;
    public static double light = 0;
    public void init(HardwareMap hm) {
        LED = hm.get(Servo.class, "shiny");
    }
    public void Loop() {
        LED.setPosition(light);
    }
}
