package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

/**
 * depot auto:
 * -lands, deposits teammarker, samples, scores sampling, parks
 * crater auto:
 * -lands, collects from crater, scores collected into lander, samples, drops team marker, parks
 */

public class jojo {
    private OpMode opMode;
    private Telemetry telemetry;
    private Robot nav;

    /**
     * The constructor method that zcontains everything to run in initialization.
     *
     * @param opMode    - The OpMode required to access motors. Often, 'this' will suffice.
     * @param telemetry - Telemetry of the current OpMode, used to output data to the screen.
     */
    public jojo(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode, org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        this.opMode = opMode;
        this.telemetry = telemetry;
        nav = new Robot(opMode, telemetry, true);
        nav.hold(0.1f);
    }

    //----Run this to run Autonomous----//
    public void runAutonomous() {
        nav.goDistanceHold(5f);
        nav.goDistanceHold(-5f);

    }

}
