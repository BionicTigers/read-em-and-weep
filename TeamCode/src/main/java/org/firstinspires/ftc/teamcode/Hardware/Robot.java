package org.firstinspires.ftc.teamcode.Hardware;


import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;


import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Movement.Location;


import java.io.File;
import java.math.BigDecimal;
import java.math.RoundingMode;

/**
 * A class for all movement methods (using PID and IMU) for Rover Ruckus for autonomous as well as mechanisms methods for autonomous as well
 * (Basically an autonomous base)
 */
public class Robot {

    //-----tweak values-----//
    //private float maximumMotorPower = 0.5f;             //when executing a goToLocation function, robot will never travel faster than this value (percentage 0=0%, 1=100%)
    private float encoderCountsPerRev = 1120f;         //encoder ticks per one revolution
    private boolean useTelemetry;                       //whether to execute the telemetry method while holding
    private float minVelocityCutoff = 0.06f;            //velocity with which to continue program execution during a hold (encoder ticks per millisecond)





    //-----misc internal values-----//
    private com.qualcomm.robotcore.eventloop.opmode.LinearOpMode hardwareGetter;
    private org.firstinspires.ftc.robotcore.external.Telemetry telemetry;

    private DcMotor velocityMotor;
    private long prevTime;
    private int prevEncoder;
    private float velocity = 0f;

    private float wheelDistance = 6.66f;                //distance from center of robot to center of wheel (inches)
    private float wheelDiameter = 4;                //diameter of wheel (inches)

    BNO055IMU imu;
    public Orientation angles;
    private Location pos = new Location();
    public int gameState = 0;

    //location of robot as [x,y,z,rot] (inches / degrees)

    //-----motors-----//
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public float beepbeep = 0;




    /**
     * The constructor class for Navigation
     *
     * @param hardwareGetter - The OpMode required to access motors. Often, 'this' will suffice.
     * @param telemetry      - Telemetry of the current OpMode, used to output data to the screen.
     * @param useTelemetry   - Whether or not to output information about stored variables and motors during hold periods.
     */
    public Robot(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode hardwareGetter, org.firstinspires.ftc.robotcore.external.Telemetry telemetry, boolean useTelemetry) {
        this.hardwareGetter = hardwareGetter;
        this.telemetry = telemetry;
        this.useTelemetry = useTelemetry;




        //-----motors-----//
        frontLeft = hardwareGetter.hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareGetter.hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareGetter.hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareGetter.hardwareMap.dcMotor.get("backRight");
        driveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        //-----velocity control-----//
        velocityMotor = frontLeft;
        prevTime = System.currentTimeMillis();
        prevEncoder = velocityMotor.getCurrentPosition();

        //---imu initialization-----//
        BNO055IMU.Parameters noots = new BNO055IMU.Parameters();
        noots.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        noots.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        noots.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        noots.loggingEnabled = true;
        noots.loggingTag = "IMU";
        noots.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareGetter.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(noots);


    }



    /**
     * Sets drive motor powers.
     *
     * @param left  power of left two motors as percentage (0-1).
     * @param right power of right two motors as percentage (0-1).
     */
    public void drivePower(float left, float right) {
        frontLeft.setPower(left);
        frontRight.setPower(right);
        backRight.setPower(right);
        backLeft.setPower(left);
    }

    /**
     * Sets drive motor target encoder to given values.
     *
     * @param left  encoder set for left motors.
     * @param right encoder set for right motors.
     */
    public void drivePosition(int left, int right) {
        frontLeft.setTargetPosition(left);
        frontRight.setTargetPosition(right);
        backRight.setTargetPosition(right);
        backLeft.setTargetPosition(left);
    }

    /**
     * Sets all drive motor run modes to given mode.
     *
     * @param mode name DcMotor mode to given value.
     */
    public void driveMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backRight.setMode(mode);
        backLeft.setMode(mode);
    }

    /**
     * Stops all drive motors and resets encoders.
     */
    public void stopAllMotors() {
        drivePower(0f, 0f);
        driveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Pseudo PID to drive the given distance.
     *
     * @param distance Distance to drive forward in inches.
     */
    public void goDistance(float distance, float maximumMotorPower, float maxiumMotorPower) {
        driveMethodSimple(-distance, distance, maximumMotorPower, maximumMotorPower);
        pos.translateLocal(distance);
    }

    /**
     * Same as goDistance() e.g. PID drive in a straight line, but with a holdForDrive() at the end. I'm not sure. Just roll with it.
     *
     * @param distance Distance to drive forward in inches.
     */

    public void goDistanceHold(float distance) {
        goDistance(distance, 0.7f, 0.7f);
        holdForDrive();
    }

    /**
     * Executes a point turn to face the given world rotation.
     *
     * @param rot Target azimuth in degrees
     */
    public void pointTurn(float rot) {
        float rota = (rot - pos.getLocation(3)) % 360f;
        float rotb = -(360f - rota);
        float optimalRotation = (Math.abs(rota) < Math.abs(rotb) ? rota : rotb); //selects shorter rotation
        float distance = (float) (Math.toRadians(optimalRotation) * wheelDistance); //arc length of turn (radians * radius)
        driveMethodSimple(distance, distance, 0.3f, 0.3f);

        pos.setRotation(rot);
    }

//    if(potentiometer.getVoltage() < desiredPosition){
//
//    }
//    else if(potentiometer.getVoltage() > desiredPosition){
//         }

    /**
     * Executes a point turn to face the given Location.
     *
     * @param loc Target Location object
     */
    public void pointTurn(Location loc) {
        pointTurn((float) Math.toDegrees(Math.atan2(loc.getLocation(2) - pos.getLocation(2), loc.getLocation(0) - pos.getLocation(0))));
    }

    /**
     * Executes a point turn relative to the current location. Positive is counterclockwise.
     *
     * @param rot the amount to rotate the robot in degrees. Positive is counterclockwise.
     */
    public void pointTurnRelative(float rot) {
        pointTurn(pos.getLocation(3) + rot);
    }

    /**
     * Executes a point turn to the given heading, first updating the position with the internal IMU value. Will holdForDrive() automatically.
     *
     * @param heading Target rotation in degrees.
     */
    public void pointTurnIMU(float heading) {
        pos.setRotation((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)).firstAngle);
        pointTurn(heading);
        holdForDrive();
    }

    /**
     * Drive method that independently controls the position and power of the left and right drive motors.
     *
     * @param distanceL float. Distance in inches for left motors to traverse.
     * @param distanceR float. Distance in inches for right motors to traverse.
     * @param LPower    float. Power percentage for left motors (0.0-1.0).
     * @param RPower    float. Power percentage for right motors (0.0-1.0).
     */
    private void driveMethodSimple(float distanceL, float distanceR, float LPower, float RPower) {
        driveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int l = (int) (distanceL / (wheelDiameter * Math.PI) * encoderCountsPerRev);
        int r = (int) (distanceR / (wheelDiameter * Math.PI) * encoderCountsPerRev);
        drivePosition(-l, -r);
        drivePower(LPower, RPower);
        driveMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Holds program execution until drive motor velocities are below the minimum cutoff.
     * Will output telemetry if class initialized with useTelemetry true.
     */
    public void holdForDrive() {
        hold(0.2f);
        gameState++;
        while (updateVelocity() > minVelocityCutoff && hardwareGetter.opModeIsActive()) {
            if (useTelemetry) telemetryMethod();
        }
    }



    /**
     * Hold program for given number of seconds.
     *
     * @param seconds float. Number of seconds to wait.
     */
    public void hold(float seconds) {
        long stopTime = System.currentTimeMillis() + (long) (seconds * 1000);
        gameState++;
        while (System.currentTimeMillis() < stopTime && hardwareGetter.opModeIsActive()) {
            if (useTelemetry) telemetryMethod();
        }
    }

    /**
     * Updates the stored velocity of the robot to reflect reality.
     *
     * @return float. New velocity in encoder ticks per millisecond.
     */
    private float updateVelocity() {
        velocity = Math.abs((float) (velocityMotor.getCurrentPosition() - prevEncoder) / (System.currentTimeMillis() - prevTime));
        prevEncoder = velocityMotor.getCurrentPosition();
        prevTime = System.currentTimeMillis();
        return velocity;
    }

    /**
     * A simple method to output the status of all motors and other variables to telemetry.
     */
    public void telemetryMethod() {
        updateVelocity();
        telemetry.addData("Party has started", "woot woot");
        telemetry.addData("Game State = ", gameState);
        String motorString = "FL = " + frontLeft.getCurrentPosition() + " BL = " + backLeft.getCurrentPosition() + " FR = " + frontRight.getCurrentPosition() + " BR = " + backRight.getCurrentPosition();
        telemetry.addData("Drive = ", motorString);

        telemetry.addData("Pos = ", pos);
        telemetry.addData("Velocity = ", velocity);

        //telemetry.addData("Pot",percentTurned())


        //   telemetry.addData("CubeXPosition",detector.getXPosition());



        telemetry.update();
    }

    /**
     * Calibrates the imu, probably best to do in init
     * May take a hot second.
     */
    public void calibrateHeading() {
        BNO055IMU.Parameters noots = new BNO055IMU.Parameters();
        noots.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        noots.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        noots.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        noots.loggingEnabled = true;
        noots.loggingTag = "IMU";
        noots.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(noots);

        BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();
        String filename = "BNO055IMUCalibration.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, calibrationData.serialize());

        pos.setRotation((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)).firstAngle);

        telemetry.update();
        telemetry.log().add("IMU: CALIBRATED", filename);
        telemetry.update();
    }

    /**
     * Compares given heading value with IMU heading value. If less than error, returns true.
     *
     * @param heading the heading to check for, heading in is degrees
     * @param err     the amount of error (in degrees) allowed to return true
     * @return boolean.
     */
    public boolean checkHeading(float heading, float err) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Math.abs(heading - angles.firstAngle) < err;
    }

    private static double round(double value) { //Allows telemetry to display nicely
        BigDecimal bd = new BigDecimal(value);
        bd = bd.setScale(3, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }
}
