package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * A class made to run the crater code pulled from AutoGeneric
 */
@Autonomous(name = "Auto Crater", group = "Auto")
public class auto extends LinearOpMode {
    public void runOpMode() {
        jojo jojo = new jojo(this, telemetry);

        while (!isStarted()) {
            telemetry.addData("cool", "waiting to start");
            telemetry.update();
        }
        //waitForStart();
        jojo.runAutonomous();
    }

    public boolean isStopping() {
        return opModeIsActive();
    }

}
