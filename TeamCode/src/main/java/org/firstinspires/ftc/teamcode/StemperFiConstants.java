package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import androidx.annotation.ColorInt;

public class StemperFiConstants {
    // Freight Frenzy
    public static final long TICKS_PER_MM = Math.round(36400 /600);
    public static final int MM_PER_SQUARE = 600;

    public static final int TICKS_PER_DEGREE = 210;

    public static final int RED = Color.rgb(200, 0, 0);
    public static final int GREEN = Color.rgb(0,200, 0);
    public static final int BLUE = Color.rgb(0,0, 200);
    public static final int OFF = Color.rgb(0,0,0);

    public static final double GRABBER_SERVO_CLOSED = 0.62; //1.0;
    public static final double GRABBER_SERVO_OPEN = 0.40;

    public static final double ANGLE_SERVO_FLAT = 0.53;
    public static final double ANGLE_SERVO_SCORE_DELTA = 0.12;

    public static final int LIFT_TICKS_PLATE = 210; // gobilda 890;
    public static final int LIFT_TICKS_LOW = 1_106; // gobilda 7_030;
    public static final int LIFT_TICKS_MED =  2_198; // gobilda 11_200;
    public static final int LIFT_TICKS_HIGH = 3_365; // gobilda 15_800;
    public static final int LIFT_TICKS_MAX = 3_465;


    public static final int LIFT_TICKS_TWO = 171; // gobilda 890;
    public static final int LIFT_TICKS_THREE = 324; // gobilda 7_030;
    public static final int LIFT_TICKS_FOUR =  490; // gobilda 11_200;
    public static final int LIFT_TICKS_FIVE = 643; // gobilda 15_800;

    // How many encoder ticks to move forward/backwards 1 cm
    public static final double TICKS_PER_CM = 1000.0/58.0;
    // Hom many encoder ticks to slide left/right 1 cm
    public static final double SLIDE_TICKS_PER_CM = 1000.0/50.0;
   // public static final double TICKS_PER_DEGREE = 1100.0/90.0;

    public static final double ROTATE_SERVO_FRONT = 0.15;
    public static final double ROTATE_SERVO_BACK = 0.81;
    public static final int ROTATE_SERVO_LIFT_THRESHOLD = LIFT_TICKS_LOW / 2;
    public static final double ROTATE_SERVO_STEP_SIZE = 0.1;

    public @ColorInt
    static final int[] DIRECTION_ACTIVE = new int[] {
            Color.rgb(0, 0, 0),
            Color.rgb(0, 0, 0),
            Color.rgb(0, 0, 0),
            Color.rgb(0, 0, 0),
            Color.rgb(0, 0, 0),
            Color.rgb(0, 200, 0),
            Color.rgb(0, 200, 0),
            Color.rgb(0, 200, 0),
            Color.rgb(0, 200, 0),
            Color.rgb(0, 200, 0)};

    public @ColorInt
    static final int[] DIRECTION_INACTIVE = new int[] {
            Color.rgb(0, 0, 0),
            Color.rgb(0, 0, 0),
            Color.rgb(0, 0, 0),
            Color.rgb(0, 0, 0),
            Color.rgb(0, 0, 0),
            Color.rgb(200, 0, 200),
            Color.rgb(200, 0, 200),
            Color.rgb(200, 0, 200),
            Color.rgb(200, 0, 200),
            Color.rgb(200, 0, 200)};
}
