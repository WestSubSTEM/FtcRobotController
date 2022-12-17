package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

public class StemperFiConstants {
    // Freight Frenzy
    public static final long TICKS_PER_MM = Math.round(36400 /600);
    public static final int MM_PER_SQUARE = 600;

    public static final int TICKS_PER_DEGREE = 210;

    public static final int RED = Color.rgb(200, 0, 0);
    public static final int GREEN = Color.rgb(0,200, 0);
    public static final int BLUE = Color.rgb(0,0, 200);
    public static final int OFF = Color.rgb(0,0,0);

    public static final double GRABBER_SERVO_CLOSED = 1.0;
    public static final double GRABBER_SERVO_OPEN = 0.7;

    public static final double ROTATE_SERVO_LIMIT = 0.1;
    public static final double ROTATE_SERVO_CENTER = 0.47;


    public static final int LIFT_TICKS_PLATE = 482; // gobilda 890;
    public static final int LIFT_TICKS_LOW = 3_488; // gobilda 7_030;
    public static final int LIFT_TICKS_MED =  5_802; // gobilda 11_200;
    public static final int LIFT_TICKS_HIGH = 8_154; // gobilda 15_800;
    public static final int LIFT_TICKS_MAX = 8_350;


    public static final int LIFT_TICKS_TWO = 433; // gobilda 890;
    public static final int LIFT_TICKS_THREE = 698; // gobilda 7_030;
    public static final int LIFT_TICKS_FOUR =  857; // gobilda 11_200;
    public static final int LIFT_TICKS_FIVE = 1_225; // gobilda 15_800;

    // How many encoder ticks to move forward/backwards 1 cm
    public static final double TICKS_PER_CM = 1000.0/58.0;
    // Hom many encoder ticks to slide left/right 1 cm
    public static final double SLIDE_TICKS_PER_CM = 1000.0/50.0;
   // public static final double TICKS_PER_DEGREE = 1100.0/90.0;

}
