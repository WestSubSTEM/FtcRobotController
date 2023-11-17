package org.firstinspires.ftc.teamcode;

public class STEMperFiConstants {

    public enum STATE {
        INTAKE,
        TRANSFER_START,
        TRANSFER_PINCH,
        TRANSFER_FLIP,
        PLACE_PIXEL,
        HANG
    };

    public static final double PLATE_INTAKE = 0.15;
    public static final double PLATE_FLAT = 0.21;
    public static final double PLATE_PINCH = 0.37;

    public static final double PLATE_ARM_INTAKE = 0.52;
    public static final double PLATE_ARM_PINCH = 0.46;

    public static final double PINCH_OPEN = 0.5;
    public static final double PINCH_CLOSED = 0.8;

    public static final double PINCH_FLIP_INTAKE = 0.62;
    public static final double PINCH_FLIP_TRANSFER = 0.39;
    public static final double PINCH_FLIP_BACKDROP = 0.0;

    public static final double PINCH_ROTATE_TRANSFER = 0.11;
    public static final double PINCH_ROTATE_HORIZONTAL = 0.41;
    public static final double PINCH_ROTATE_LEFT = 0.55;
    public static final double PINCH_ROTATE_RIGHT = 0.3 ;

    public static final int LIFT_TARGET_HANG = 0;
    public static final int LIFT_TARGET_TRANSFER = 0;
    public static final int LIFT_TARGET_INTAKE = 0;
    public static final int LIFT_TARGET_FLIP = 450;

    public static final double INTAKE_SPEED = 0.4;
}
