package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Movement.Location;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

public class Robot {
    public RobotType robotType;
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

    /**
     * TODO add a function that checks the config.xml file
     */
    private ArrayList<DcMotorEx> rightMotors;

    /**
     * Constructor 1
     *
     * @param type
     */
    public Robot(RobotType type, Location loc, HardwareMap hw) {
        Motor1 = (DcMotorEx) hw.dcMotor.get("forntLeft");
        Motor2 = (DcMotorEx) hw.dcMotor.get("backLeft");
        Motor3 = (DcMotorEx) hw.dcMotor.get("frontLeft");
        Motor4 = (DcMotorEx) hw.dcMotor.get("backLeft");
        robotType = type;
        robot = loc;
        driveMotors = new ArrayList<DcMotorEx>(Arrays.asList(Motor1, Motor2, Motor3, Motor4));
        leftMotors = new ArrayList<DcMotorEx>(Arrays.asList(Motor1, Motor2));
        rightMotors = new ArrayList<DcMotorEx>(Arrays.asList(Motor3, Motor4));
        for(DcMotorEx motorEx: driveMotors){
            motorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorEx.setTargetPositionTolerance(50);
        }
    }

    public void forwardInches(double distance) throws UnsupportedOperationException {
        if (robotType == RobotType.DIFFY_MECH) {
            if (!Motor1.isBusy()) {
                for (DcMotorEx motorEx : driveMotors) {
                    motorEx.setPower(0);
                    motorEx.setTargetPosition(motorEx.getTargetPosition() + (int) (distance / (4 * Math.PI) * RobotValues.twentyTicksPerRev));
                }
                for (DcMotorEx motorEx : driveMotors) {
                    motorEx.setPower(.5);
                }
            }
        } else {
            throw new UnsupportedOperationException("This robot hasn't been coded yet. So go yell at chris or code it yourself ;)");
        }

    }

    public void turnDegrees(double degrees) throws UnsupportedOperationException {
        if (!Motor1.isBusy() && !Motor4.isBusy()) {
            if (robotType == RobotType.DIFFY_MECH) {
                for (DcMotorEx motorEx : leftMotors) {
                    motorEx.setPower(.5);
                    motorEx.setTargetPosition(motorEx.getTargetPosition() + (int) (RobotValues.distFromCenter * 2 * Math.PI * (degrees / 360)));
                }
                for (DcMotorEx motorEx : rightMotors) {
                    motorEx.setPower(.5);
                    motorEx.setTargetPosition(motorEx.getTargetPosition() + (int) -(RobotValues.distFromCenter * 2 * Math.PI * (degrees / 360)));
                }
            }
        } else {
            throw new UnsupportedOperationException("This robot hasn't been coded yet. So go yell at chris or code it yourself ;)");
        }

    }

    /**
     * TODO add more constructors at different levels of control
     */
    public enum RobotType {
        BOARD,
        LUCY_6W_DRIVE,
        DIFFY_MECH


    }
}
