package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import androidx.annotation.ColorInt;

public class STEMperFiConstants {


    public static final @ColorInt int COLOR_RED = Color.parseColor("red");
    public static final @ColorInt int COLOR_ORANGE = Color.parseColor("orange");
    public static final @ColorInt int COLOR_YELLOW = Color.parseColor("yellow");
    public static final @ColorInt int COLOR_GREEN = Color.parseColor("green");
    public static final @ColorInt int COLOR_BLUE = Color.parseColor("blue");
    public static final @ColorInt int COLOR_PURPLE = Color.parseColor("purple");

    public enum STATE {
        INTAKE,
        TRANSFER_START,
        TRANSFER_PINCH,
        TRANSFER_FLIP,
        DRIVE_TO_HUMAN,
        PLACE_PIXEL,
        HANG,
        INTAKE_PREP
    };
    

    public static final double PINCH_OPEN = 0.5;
    public static final double PINCH_CLOSED = 0.85;

    public static final double PINCH_FLIP_INTAKE = 0.41;
    public static final double PINCH_FLIP_TRANSFER = 0.41;
    public static final double PINCH_FLIP_HANG = 0.1;

    public static final double PINCH_FLIP_BACKDROP = 0.19;

    // vertical 0.11 &  0.71
    public static final double PINCH_ROTATE_VERTICAL = 0.71;

    //  Horizontal 1.0 & .4
    public static final double PINCH_ROTATE_HORIZONTAL = 0.41;

    // add sub .1 from horizontal
    public static final double PINCH_ROTATE_LEFT = 0.55;
    public static final double PINCH_ROTATE_RIGHT = 0.3 ;

    public static final int LIFT_TARGET_HANG = 2000;
    public static final int LIFT_TARGET_PINCH = 0;
    public static final int LIFT_TARGET_INTAKE = 100;
    public static final int LIFT_TARGET_FLIP = 450;

    public static final double INTAKE_SPEED = 0.4;

    public static final long TICKS_PER_MM = Math.round(36400 /600);
    public static final int MM_PER_SQUARE = 600;

    public static final int TICKS_PER_DEGREE = 210;
    // How many encoder ticks to move forward/backwards 1 cm
    public static final double TICKS_PER_CM = 1000.0/58.0;
    // Hom many encoder ticks to slide left/right 1 cm
    public static final double SLIDE_TICKS_PER_CM = 1000.0/50.0;
// public static final double TICKS_PER_DEGREE = 1100.0/90.0;
}
