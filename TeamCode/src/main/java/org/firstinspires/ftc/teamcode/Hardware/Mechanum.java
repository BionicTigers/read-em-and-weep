package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Movement.Location;
import org.firstinspires.ftc.teamcode.Hardware.Robot.*;

public class Mechanum extends Robot{

    /**
     * Constructor 1
     *
     * @param loc
     */
    public Mechanum(Location loc, Telemetry telemetry, HardwareMap hw) {
        super(telemetry, loc,hw);
    }

    /**
     * TODO superload this method with one that does this with power
     * @param distance
     * @param maximumMotorPower
     * @param maxiumMotorPower
     */
    public void goDistance(float distance, float maximumMotorPower, float maxiumMotorPower) {
        forwardInches(distance);
    }
    public void goDistanceHold(float distance) {
        goDistance(distance, 0.7f, 0.7f);
        holdForDrive();
    }
    public void turnDegrees(double degrees) throws UnsupportedOperationException {
        if (!Motor1.isBusy() && !Motor4.isBusy()) {

                for (DcMotorEx motorEx : leftMotors) {
                    motorEx.setPower(.5);
                    motorEx.setTargetPosition(motorEx.getTargetPosition() + (int) (RobotValues.distFromCenter * 2 * Math.PI * (degrees / 360)));
                }
                for (DcMotorEx motorEx : rightMotors) {
                    motorEx.setPower(.5);
                    motorEx.setTargetPosition(motorEx.getTargetPosition() + (int) -(RobotValues.distFromCenter * 2 * Math.PI * (degrees / 360)));
                }
            }

    }
    public void forwardInches(double distance) throws UnsupportedOperationException {

            for (DcMotorEx motorEx : driveMotors) {
                motorEx.setPower(0);
                motorEx.setTargetPosition(motorEx.getTargetPosition() + (int) (distance / (4 * Math.PI) * RobotValues.twentyTicksPerRev));
            }
            for (DcMotorEx motorEx : driveMotors) {
                motorEx.setPower(.5);
            }
    }

}
