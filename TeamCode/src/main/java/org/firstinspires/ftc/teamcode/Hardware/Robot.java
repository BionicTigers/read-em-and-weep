package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Movement.Location;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

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

    RevBulkData bulkData;
    AnalogInput a0, a1, a2, a3;
    DigitalChannel d0, d1, d2, d3, d4, d5, d6, d7;
    ExpansionHubMotor motor0, motor1, motor2, motor3;
    ExpansionHubEx expansionHub;

    //Location of the bot
    private Location robot;
    //Array of different types of things
    private ArrayList<DcMotorEx> driveMotors;
    private ArrayList<DcMotorEx> leftMotors;
    //This array should go left encoder, right encoder, back encoder
    private ArrayList<DcMotorEx> encoders;
    private int[] encoderPosition = {0,0,0};

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
        Motor1 = (DcMotorEx) hw.dcMotor.get("frontLeft");
        Motor2 = (DcMotorEx) hw.dcMotor.get("backLeft");
        Motor3 = (DcMotorEx) hw.dcMotor.get("frontRight");
        Motor4 = (DcMotorEx) hw.dcMotor.get("backRight");
        expansionHub = hw.get(ExpansionHubEx.class, "Expansion Hub 2");
        robotType = type;
        robot = loc;
        driveMotors = new ArrayList<DcMotorEx>(Arrays.asList(Motor1, Motor2, Motor3, Motor4));
        leftMotors = new ArrayList<DcMotorEx>(Arrays.asList(Motor1, Motor2));
        rightMotors = new ArrayList<DcMotorEx>(Arrays.asList(Motor3, Motor4));
        encoders = new ArrayList<DcMotorEx>(Arrays.asList(Motor1,Motor2,Motor3));
        for (DcMotorEx motorEx : driveMotors) {
            motorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorEx.setTargetPositionTolerance(50);
        }
    }

    /**
     * TODO add something that periodically checks the camera
     */
    public void updatePosition(){
        bulkData = expansionHub.getBulkInputData();
        double[] encoderDeltamm = new double[3];
        for(int i = 0;i<3;i++){
           encoderDeltamm[i] =  RobotValues.odoDiamMM*Math.PI*((encoderPosition[i]-bulkData.getMotorCurrentPosition(i))/RobotValues.twentyTicksPerRev);
           encoderPosition[i]=bulkData.getMotorCurrentPosition(i);
        }
        double botRotDelta = (encoderDeltamm[0]-encoderDeltamm[1])/RobotValues.odoDistBetweenMM;
        double robotXDelta = encoderDeltamm[2]-RobotValues.middleOdoFromMiddleMM*botRotDelta;
        double robotYDelta = (encoderDeltamm[0]-encoderDeltamm[1])/2;
        robot.translateLocal(robotYDelta,robotXDelta,botRotDelta);

    }

    public void forwardInches(double distance) throws UnsupportedOperationException {
        if (robotType == RobotType.DIFFY_MECH) {
            for (DcMotorEx motorEx : driveMotors) {
                motorEx.setPower(0);
                motorEx.setTargetPosition(motorEx.getTargetPosition() + (int) (distance / (4 * Math.PI) * RobotValues.twentyTicksPerRev));
            }
            for (DcMotorEx motorEx : driveMotors) {
                motorEx.setPower(.5);
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
