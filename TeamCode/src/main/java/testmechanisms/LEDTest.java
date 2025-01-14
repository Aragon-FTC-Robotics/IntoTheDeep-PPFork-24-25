package testmechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LEDTest {
    Servo LED;
    public static double light = 0;
    public void init(HardwareMap hm) {
        LED = hm.get(Servo.class, "LED");
    }
    public void Loop() {
        LED.setPosition(light);
    }
}
