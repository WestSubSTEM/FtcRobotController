package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import androidx.annotation.ColorInt;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "LED TEST", group = "Test")
@Disabled
public class TestLEDOpMode extends OpMode {
    private QwiicLEDStrip ledStripFront;
    private QwiicLEDStrip ledStripBack;
    private ElapsedTime elapsedTime = new ElapsedTime();
    private int colorIndex = 0;
    private @ColorInt int[] colors = new int[] {
            Color.rgb(148, 0, 211),
            Color.rgb(75, 0, 130),
            Color.rgb(0, 0, 255),
            Color.rgb(0, 255, 0),
            Color.rgb(255, 0, 0),
            Color.rgb(255, 255, 0),
            Color.parseColor("purple"),
            Color.parseColor("teal"),
            Color.parseColor("silver"),
            Color.rgb(0, 0, 0) };

    private @ColorInt int[] direction = new int[] {
            Color.rgb(0, 0, 0),
            Color.rgb(0, 0, 0),
            Color.rgb(0, 0, 0),
            Color.rgb(0, 0, 0),
            Color.rgb(0, 0, 0),
            Color.rgb(0, 210, 0),
            Color.rgb(0, 210, 0),
            Color.rgb(0, 210, 0),
            Color.rgb(0, 210, 0),
            Color.rgb(0, 210, 0)};

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        ledStripFront = hardwareMap.get(QwiicLEDStrip.class, "led");
        ledStripFront.setBrightness(2);
        ledStripFront.setColor(Color.parseColor("purple"));
//        ledStripBack = hardwareMap.get(QwiicLEDStrip.class, "led_strip_back");
//        ledStripBack.setBrightness(2);
//        ledStripBack.setColor(Color.parseColor("blue"));
    }

    @Override
    public void start() {
        elapsedTime.reset();
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        if (elapsedTime.milliseconds() >= 500) {
//            if (colorIndex == 1) {
//                ledStripFront.turnAllOff();
////                ledStripBack.setColors(direction);
//                colorIndex = 0;
//            } else {
//                ledStripFront.setColors(direction);
//  //              ledStripBack.turnAllOff();
//                colorIndex++;
//            }
            if (colorIndex == colors.length) {
                ledStripFront.setColors(colors);
                colorIndex = 0;
            } else {
                ledStripFront.setColor(colors[colorIndex]);
                colorIndex++;
            }
            elapsedTime.reset();
        }
    }

    @Override
    public void stop() {
        ledStripFront.turnAllOff();
        ledStripBack.turnAllOff();
    }
}