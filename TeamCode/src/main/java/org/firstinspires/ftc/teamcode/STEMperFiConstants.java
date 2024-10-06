package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import androidx.annotation.ColorInt;

public class STEMperFiConstants {


    public static final @ColorInt int COLOR_RED = Color.rgb(250, 0, 0);
    public static final @ColorInt int COLOR_ORANGE =  Color.rgb(250, 165, 0);//Color.parseColor("orange");
    public static final @ColorInt int COLOR_YELLOW = Color.rgb(250,250,200);// Color.parseColor("yellow");
    public static final @ColorInt int COLOR_GREEN = Color.rgb(0,250,0);
    public static final @ColorInt int COLOR_BLUE = Color.rgb(0,0,250);
    public static final @ColorInt int COLOR_PURPLE = Color.rgb(128,0,128);

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

    public enum TeamPropColor {
        BLUE,
        RED,
        UNKNOWN
    }
    
    public static final double DRONE_CLOSE = .75;
    public static final double DRONE_LAUNCH = 1;

    public static final double PINCH_OPEN = 0.7;
    public static final double PINCH_CLOSED = 0.05;
    public static final double PINCH_CLOSED_WALL = 0.05;

    public static final double PINCH_FLIP_INTAKE = 0.24; //0.22; //0.43; //0.39;
    public static final double PINCH_FLIP_TRANSFER = 0.24; //0.22; //0.39;
    public static final double PINCH_FLIP_HANG = 0.85; //0.13; //0.09;

    public static final double PINCH_FLIP_BACKDROP = 0.62; //0.16;

    // vertical 0.11 &  0.71
    public static final double PINCH_ROTATE_VERTICAL = .68;

    //  Horizontal 1.0 & .4
    public static final double PINCH_ROTATE_HORIZONTAL = 0.36;

    // add sub .1 from horizontal
    public static final double PINCH_ROTATE_LEFT = 0.4;
    public static final double PINCH_ROTATE_RIGHT = 0.2;

    public static final int LIFT_TARGET_MAX = 2000;

    public static final int LIFT_TARGET_HANG = 1400;
    public static final int LIFT_TARGET_PINCH = 15;
    public static final int LIFT_TARGET_INTAKE = 200;
    public static final int LIFT_TARGET_FLIP = 450;

    public static final int AUTO_SCORE = 580;
    public static final double INTAKE_SPEED = 0.5;

    public static final double FLIP_INCREMENT = 0.01;
    public static final long TICKS_PER_MM = Math.round(36400 /600);
    public static final int MM_PER_SQUARE = 600;

    public static final int TICKS_PER_DEGREE = 210;
    // How many encoder ticks to move forward/backwards 1 cm
    public static final double TICKS_PER_CM = 1000.0/58.0;
    // Hom many encoder ticks to slide left/right 1 cm
    public static final double SLIDE_TICKS_PER_CM = 1000.0/50.0;
// public static final double TICKS_PER_DEGREE = 1100.0/90.0;
}
