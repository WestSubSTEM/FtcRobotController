package org.firstinspires.ftc.teamcode;

public class STEMperFiConstants {

    public enum STATE {
        INTAKE,
        TRANSFER_START,
        TRANSFER_PINCH,
        TRANSFER_FLIP,
        PLACE_PIXEL,
        HANG,
        INTAKE_PREP
    };
    
    public static final double PLATE_INTAKE = 0.15;
    public static final double PLATE_FLAT = 0.21;
    public static final double PLATE_PINCH = 0.35;

    public static final double PLATE_ARM_INTAKE = 0.52;
    public static final double PLATE_ARM_PINCH = 0.46;

    public static final double PINCH_OPEN = 0.5;
    public static final double PINCH_CLOSED = 0.9;

    public static final double PINCH_FLIP_INTAKE = 0.62;
    public static final double PINCH_FLIP_TRANSFER = 0.38;
    public static final double PINCH_FLIP_HANG = 0.3;

    public static final double PINCH_FLIP_BACKDROP = 0.0;

    public static final double PINCH_ROTATE_TRANSFER = 0.72;
    public static final double PINCH_ROTATE_HORIZONTAL = 0.41;
    public static final double PINCH_ROTATE_LEFT = 0.55;
    public static final double PINCH_ROTATE_RIGHT = 0.3 ;

    public static final int LIFT_TARGET_HANG = 2000;
    public static final int LIFT_TARGET_PINCH = 260;
    public static final int LIFT_TARGET_INTAKE = 0;
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
