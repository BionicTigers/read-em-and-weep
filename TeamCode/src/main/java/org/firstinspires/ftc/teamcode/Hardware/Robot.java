package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Movement.Location;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

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
    private ArrayList<DcMotorEx> driveMotors;
    private ArrayList<DcMotorEx> leftMotors;
    private ArrayList<DcMotorEx> rightMotors;

    /**
     * TODO add a function that checks the config.xml file
      */
    /**
     * TODO add more constructors at different levels of control
     */
    public enum RobotType{
        BOARD,
        LUCY_6W_DRIVE,
        DIFFY_MECH


    }
    public RobotType robotType;
    /**
     * Constructor 1
     * @param type
     */
    public Robot(RobotType type, Location loc){
        robotType = type;
        robot = loc;
        driveMotors = new ArrayList<DcMotorEx>(Arrays.asList(Motor1,Motor2,Motor3,Motor4));
        leftMotors = new ArrayList<DcMotorEx>(Arrays.asList(Motor1,Motor2));
        rightMotors = new ArrayList<DcMotorEx>(Arrays.asList(Motor3,Motor4));
    }

    public void forwardInches(double distance)throws UnsupportedOperationException{
        if (robotType == RobotType.DIFFY_MECH){
            for (DcMotorEx motorEx: driveMotors ){
                motorEx.setPower(.5);
                motorEx.setTargetPosition((int)((distance/(Math.PI*4)*RobotValues.twentyTicksPerRev)));
            }
        }
        else {
            throw new UnsupportedOperationException("This robot hasn't been coded yet. So go yell at chris or code it yourself ;)");
        }

    }
    public void turnDegrees(double degrees)throws UnsupportedOperationException{
        if (robotType == RobotType.DIFFY_MECH){
            for (DcMotorEx motorEx: leftMotors ){
                motorEx.setPower(.5);
                motorEx.setTargetPosition(motorEx.getTargetPosition()+(int)(RobotValues.distFromCenter*2*Math.PI*(degrees/360)));
            }
            for(DcMotorEx motorEx: rightMotors){
                motorEx.setPower(.5);
                motorEx.setTargetPosition(motorEx.getTargetPosition()+(int)-(RobotValues.distFromCenter*2*Math.PI*(degrees/360)));
            }
        }
        else {
            throw new UnsupportedOperationException("This robot hasn't been coded yet. So go yell at chris or code it yourself ;)");
        }

    }
}
