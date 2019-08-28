package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Mechanum;
import org.firstinspires.ftc.teamcode.Movement.Location;

@TeleOp(name = "Mechanum testing")
public class MechanumTestOpMode extends LinearOpMode {
    public Mechanum drivetrain;
    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Mechanum(new Location(), hardwareMap);
        waitForStart();
        drivetrain.forwardInches(3);
        while(!isStopRequested()){}
    }
}
