package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Flat", group = "State")
public class TeleMeetQualFlat extends TeleMeetQual {

    @Override
    public void init() {
        super.init();
        HIGH = StemperFiConstants.LIFT_TICKS_HIGH_FLAT;
        MED = StemperFiConstants.LIFT_TICKS_MED_FLAT;
        LOW = StemperFiConstants.LIFT_TICKS_LOW_FLAT;
        PLATE = StemperFiConstants.LIFT_TICKS_PLATE_FLAT;
    }

    @Override
    public void loop() {
        super.loop();
        angleServo.setPosition(StemperFiConstants.ANGLE_SERVO_FLAT);
    }
}