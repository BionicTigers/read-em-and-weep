package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Robot {
    //Declaration of our 8 DC motors
    DcMotorEx Motor1;
    DcMotorEx Motor2;
    DcMotorEx Motor3;
    DcMotorEx Motor4;
    DcMotorEx Motor5;
    DcMotorEx Motor6;
    DcMotorEx Motor7;
    DcMotorEx Motor8;

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
    public Robot(RobotType type){

    }
}
