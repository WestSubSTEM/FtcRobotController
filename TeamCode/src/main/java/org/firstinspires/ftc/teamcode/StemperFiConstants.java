package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

public class StemperFiConstants {
    // Freight Frenzy
    public static final long TICKS_PER_MM = Math.round(36400 /600);
    public static final int MM_PER_SQUARE = 600;

    public static final int TICKS_PER_DEGREE = 1000 / 10;

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

    public static final double BUCKET_SERVO_RIGHT_GOAL_TOP_AUTO = .4;
    public static final double BUCKET_SERVO_RIGHT_GOAL_TOP = .35;
    public static final int WORM_MOTOR_GOAL_TOP = -3759;

    public static final double BUCKET_SERVO_RIGHT_GOAL_MIDDLE = 0.0849;
    public static final int WORM_MOTOR_GOAL_MIDDLE = -5282;

    public static final double BUCKET_SERVO_RIGHT_GOAL_BOTTOM = 0.74;
    public static final int WORM_MOTOR_GOAL_BOTTOM = -4475;


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
}
