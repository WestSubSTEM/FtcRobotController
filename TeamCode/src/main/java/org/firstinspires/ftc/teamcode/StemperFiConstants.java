package org.firstinspires.ftc.teamcode;

public class StemperFiConstants {
    // How many encoder ticks to move forward/backwards 1 cm
    public static final double TICKS_PER_CM = 1000.0/58.0;
    // Hom many encoder ticks to slide left/right 1 cm
    public static final double SLIDE_TICKS_PER_CM = 1000.0/50.0;
    public static final double TICKS_PER_DEGREE = 1100.0/90.0;

    public static final double TRIGGER_SERVO_LOAD = .85;
    public static final double TRIGGER_SERVO_FIRE = .53;

    public static final double FLYWHEEL_RUN = 1.0;
    public static final double FLYWHEEL_STOP = 0;

    public static final double INTAKE_SERVO_IN = 0;
    public static final double INTAKE_SERVO_STOP = 0.5;
    public static final double INTAKE_SERVO_OUT = 1.0;

    public static final double INTAKE_MOTOR_IN = 1.0;
    public static final double INTAKE_MOTOR_STOP = 0;
    public static final double INTAKE_MOTOR_OUT = -1.0;

    public static final int WOBBLE_IN = 0;
    public static final int WOBBLE_UP = -350;
    public static final int WOBBLE_OUT = -1000;

    public static final double WOBBLE_SERVO_OPEN = 0;
    public static final double WOBBLE_SERVO_CLOSE = 0.59;
}
