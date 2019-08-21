package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Movement.Location;

public class Robot {
    //Declaration of our 8 DC motors
    private DcMotorEx Motor1;
    private DcMotorEx Motor2;
    private DcMotorEx Motor3;
    private DcMotorEx Motor4;
    private DcMotorEx Motor5;
    private DcMotorEx Motor6;
    private DcMotorEx Motor7;
    private DcMotorEx Motor8;
    //Location of the bot
    private Location robot;

    /**
     * TODO add a function that checks the config.xml file
      */
    /**
     * TODO add more constructors at different levels of control
     */
    enum RobotType{
        BOARD,
        LUCY_6W_DRIVE

    }

    /**
     * Constructor 1
     * @param type
     */
    public Robot(RobotType type, Location loc){
        robot = loc;
    }
}
