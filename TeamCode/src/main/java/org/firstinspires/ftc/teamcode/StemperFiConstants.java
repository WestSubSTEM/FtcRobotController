package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import androidx.annotation.ColorInt;

public class StemperFiConstants {


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

    public static final double DRONE_CLOSE = .44;
    public static final double DRONE_LAUNCH = .7;

    public static final double PINCH_OPEN = 0.5;
    public static final double PINCH_CLOSED = 0.79;
    public static final double PINCH_CLOSED_WALL = 0.86;

    public static final double PINCH_FLIP_INTAKE = 0.43; //0.39;
    public static final double PINCH_FLIP_TRANSFER = 0.43; //0.39;
    public static final double PINCH_FLIP_HANG = 0.13; //0.09;

    public static final double PINCH_FLIP_BACKDROP = 0.2; //0.16;

    // vertical 0.11 &  0.71
    public static final double PINCH_ROTATE_VERTICAL = .075;

    //  Horizontal 1.0 & .4
    public static final double PINCH_ROTATE_HORIZONTAL = 0.41;

    // add sub .1 from horizontal
    public static final double PINCH_ROTATE_LEFT = 0.55;
    public static final double PINCH_ROTATE_RIGHT = 0.3 ;

    public static final int LIFT_TARGET_MAX = 2000;

    public static final int LIFT_TARGET_HANG = 1400;
    public static final int LIFT_TARGET_PINCH = 15;
    public static final int LIFT_TARGET_INTAKE = 200;
    public static final int LIFT_TARGET_FLIP = 450;

    public static final double INTAKE_SPEED = 0.5;

    public static final double FLIP_INCREMENT = 0.01;
    public static final long TICKS_PER_MM = Math.round(36400 /600);
    public static final int MM_PER_SQUARE = 600;

    public static final int RED = Color.rgb(200, 0, 0);
    public static final int GREEN = Color.rgb(0,200, 0);
    public static final int BLUE = Color.rgb(0,0, 200);
    public static final int OFF = Color.rgb(0,0,0);

    public static final double INTAKE_SERVO_SPEED_IN = 0;
    public static final double INTAKE_SERVO_SPEED_OUT = 1.0;
    public static final double INTAKE_SERVO_SPEED_OFF = 0.5;

    public static final double TURNTABLE_MOTOR_OFF = 0;
    public static final double TURNTABLE_MOTOR_ON = 0.5;

    public static final int TURNTABLE_MOTOR_INITIAL_SPEED = 1000;
    public static final int TURNTABLE_INCREMENT = 100;

    public static final double ENCODER_SERVO_TELE_RIGHT = 0.5;
    public static final double ENCODER_SERVO_TELE_LEFT = 0.78;
    public static final double ENCODER_SERVO_TELE_CENTER = 0.75;

    public static final double ENCODER_SERVO_AUT0_RIGHT = 0.8;
    public static final double ENCODER_SERVO_AUT0_LEFT = 0.4;
    public static final double ENCODER_SERVO_AUT0_CENTER = 0.42;

    public static final double BUCKET_SERVO_RIGHT_MIN = 0.0;
    public static final double BUCKET_SERVO_RIGHT_MAX = 1;
    public static final double BUCKET_SERVO_RIGHT_INIT = .88;

    public static final double BUCKET_SERVO_RIGHT_INTAKE =.21;

    public static final int WORM_MOTOR_INTAKE = -183;

    public static final double BUCKET_SERVO_RIGHT_GOAL_TOP_AUTO = .55;
    public static final double BUCKET_SERVO_RIGHT_GOAL_TOP = .35;
    public static final int WORM_MOTOR_GOAL_TOP = -3759;

    public static final double BUCKET_SERVO_RIGHT_GOAL_MIDDLE = 0.0849;
    public static final int WORM_MOTOR_GOAL_MIDDLE = -5282;

    public static final double BUCKET_SERVO_RIGHT_GOAL_BOTTOM = 0.74;
    public static final int WORM_MOTOR_GOAL_BOTTOM = -4475;

    public static final int TICKS_PER_DEGREE = 210;
    // How many encoder ticks to move forward/backwards 1 cm
    public static final double TICKS_PER_CM = 1000.0/58.0;
    // Hom many encoder ticks to slide left/right 1 cm
    public static final double SLIDE_TICKS_PER_CM = 1000.0/50.0;
// public static final double TICKS_PER_DEGREE = 1100.0/90.0;

    public static final double TRIGGER_SERVO_LOAD = .85;
    public static final double TRIGGER_SERVO_FIRE = .53;

    public static final double FLYWHEEL_RUN = 1.0;
    public static final double FLYWHEEL_STOP = 0;

    public static final double INTAKE_MOTOR_IN = 1.0;
    public static final double INTAKE_MOTOR_STOP = 0;
    public static final double INTAKE_MOTOR_OUT = -1.0;

    public static final int WOBBLE_IN = 0;
    public static final int WOBBLE_UP = -350;
    public static final int WOBBLE_OUT = -1000;

    public static final double WOBBLE_SERVO_OPEN = 0;
    public static final double WOBBLE_SERVO_CLOSE = 0.59;

    public static final double MID_SERVO_SCORE = 0.3099;
    public static final double BOTTOM_SERVO_SCORE = 0.1749;
    public static final int BOTTOM_WORM_GEAR = - 6189;


}
