package mechanisms;

import android.util.Log;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import java.util.Objects;

public class Colorsensor {
    NormalizedColorSensor colorSensor;
    NormalizedRGBA sensedcolors;

    public float[] colorHSV = {0, 0, 0}; //defaut
    float gain = 30.0f;
    public float[] redHigherHSV = {0, 1, 1}; //(Hue, Saturation, Value)
    public float[] redLowerHSV = {0, 0.4f, 0.4f};
    public float[] yellowHigherHSV = {60, 1, 1};
    public float[] yellowLowerHSV = {45, 0.4f, 0.4f};
    public float[] blueHigherHSV = {300, 1, 1};
    public float[] blueLowerHSV = {240, 0.4f, 0.4f};
    public void init(HardwareMap hm) {
        colorSensor = hm.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor.setGain(gain);
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
    }
    public void Loop(){
        sensedcolors = colorSensor.getNormalizedColors();

        float r = sensedcolors.red;
        float g = sensedcolors.green;
        float b = sensedcolors.blue;

        if (Objects.nonNull(sensedcolors)) {
            Log.d("colorsensor", "Raw RGBA: Red=" + sensedcolors.red + ", Green=" + sensedcolors.green + ", Blue=" + sensedcolors.blue);
        }

        Log.d("colorsensor","Converted HSV: Hue=" + colorHSV[0] + ", Saturation=" + colorHSV[1] + ", Value=" + colorHSV[2]);

        // Convert RGB to HSV
        colorHSV = rgbToHsv(r, g, b);
    }
    public float[] getColor() {
        return new float[] {colorHSV[0],colorHSV[1],colorHSV[2]};
    }
    public boolean sensorIsRed() {
        return colorInRange(colorHSV,redLowerHSV,redHigherHSV);
    }
    public boolean sensorIsYellow() {
        return colorInRange(colorHSV,yellowLowerHSV,yellowHigherHSV);
    }
    public boolean sensorIsBlue() {
        return colorInRange(colorHSV,blueLowerHSV,blueHigherHSV);
    }
    public boolean colorInRange(float[] color, float[] min, float[] max) {
        return
                min[0] <= color[0] && color[0] <= max[0] && //Red is within min and max range
                min[1] <= color[1] && color[1] <= max[1] && //Green is within min and max range
                min[2] <= color[2] && color[2] <= max[2];   //brue is sithin the range,
    }

    // Function to convert RGB to HSV
    public float[] rgbToHsv(float r, float g, float b) {
        float max = Math.max(r, Math.max(g, b));
        float min = Math.min(r, Math.min(g, b));
        float delta = max - min;

        float h = 0, s = 0, v = max; // set Value

        if (delta != 0) {
            //calc Saturation
            s = delta / max;

            //calc Hue
            if (r == max) {
                h = (g - b) / delta;
            } else if (g == max) {
                h = 2 + (b - r) / delta;
            } else {
                h = 4 + (r - g) / delta;
            }
            h *= 60; // comvert to 360

            if (h < 0) { // make sure it's always positive
                h += 360;
            }
        }
        Log.d("colorsensor", "RGB to HSV: R=" + r + ", G=" + g + ", B=" + b + " => H=" + h + ", S=" + s + ", V=" + v);

        return new float[] {h, s, v};
    }
}
