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
    public static float gain = 50.0f;
    public float[] redHigherHighHSV = {41, 1, 1}; //(Hue, Saturation, Value) //26
    public float[] redHigherLowHSV = {11, 0.6f, 0.6f};

    public float [] redLowerHighHSV = {80, 0.9f, 0.4f}; //64, 38 near comp field
    public float[] redLowerLowHSV = {0, 0.4f, 0.05f};

    public float[] yellowHigherHSV = {97, 0.9f, 0.7f}; //82
    public float[] yellowLowerHSV = {67, 0.55f, 0.1f};

    public float[] blueHigherHSV = {230, 0.9f, 0.4f};
    public float[] blueLowerHSV = {200, 0.55f, 0.08f}; //215
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
        // Convert RGB to HSV
        colorHSV = rgbToHsv(r, g, b);
        Log.d("colorsensor","Converted HSV: Hue=" + colorHSV[0] + ", Saturation=" + colorHSV[1] + ", Value=" + colorHSV[2]);
    }
    public float[] getColor() {
        return new float[] {colorHSV[0],colorHSV[1],colorHSV[2]};
    }
    public boolean sensorIsRed() {
        return colorInRange(colorHSV,redLowerLowHSV,redLowerHighHSV) || colorInRange(colorHSV, redHigherLowHSV, redHigherHighHSV);
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
