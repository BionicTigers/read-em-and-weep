package org.firstinspires.ftc.teamcode.OpModes.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.RobotValues;
import org.firstinspires.ftc.teamcode.Movement.Trajectory;

import java.util.ArrayList;

@TeleOp(name = "Traj Testing")
public class TestingOpMode extends OpMode {
    ArrayList<ArrayList<Float>> traj;
    private DcMotorEx frontLeft;
    private double velocity;
    private float speed;
    private ElapsedTime elMili;
    private double start;
    private int startEncoderPosition;
    private float positionError;
    private float t;
    private Telemetry dashboardTelemetry;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        Trajectory trajectory = new Trajectory(1000,10000,1000,80f);
        traj = trajectory.getTrajectory();
        frontLeft = (DcMotorEx) hardwareMap.get("frontLeft");
        elMili = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    @Override
    public void loop() {
        if (RobotValues.WTFAREWEEVENDOING == 0){
        start = elMili.time();
        startEncoderPosition = frontLeft.getCurrentPosition();}

        if (RobotValues.WTFAREWEEVENDOING == 1 && (elMili.time() - start) < traj.size()) {


                dashboardTelemetry.addData("frontLeft Speeed", frontLeft.getVelocity());
                dashboardTelemetry.addData("What it should be ", speed);
                dashboardTelemetry.addData("Velocity error", speed - frontLeft.getVelocity());
                dashboardTelemetry.update();
                t = (int) (elMili.time() - start);

                    speed = traj.get((int) (elMili.time() - start)).get(1);
                    telemetry.addData("velocity", speed);
                    telemetry.update();
                    frontLeft.setVelocity(speed);



//                if(t< traj.size()*2){
//                    speed = traj.get((int)( traj.size()-((elMili.time() - start)-traj.size()))).get(1);
//                    telemetry.addData("velocity", speed);
//                    telemetry.update();
//                    frontLeft.setVelocity(speed);
//
//
            }
        if (RobotValues.WTFAREWEEVENDOING == 1 && (elMili.time() - start) > traj.size() &&(elMili.time() - start)<traj.size()*2 ) {


            dashboardTelemetry.addData("frontLeft Speeed", frontLeft.getVelocity());
            dashboardTelemetry.addData("What it should be ", speed);
            dashboardTelemetry.addData("Velocity error", speed - frontLeft.getVelocity());
            dashboardTelemetry.update();


            speed = traj.get((int)((traj.size()-1)-(elMili.time()-(start+t)))).get(1);
            telemetry.addData("velocity", speed);
            telemetry.update();
            frontLeft.setVelocity(speed);


//                if(t< traj.size()*2){
//                    speed = traj.get((int)( traj.size()-((elMili.time() - start)-traj.size()))).get(1);
//                    telemetry.addData("velocity", speed);
//                    telemetry.update();
//                    frontLeft.setVelocity(speed);
//
//
        }

        if (RobotValues.WTFAREWEEVENDOING == 2) {
            if ((int) (elMili.time()) - start < traj.size()) {
                positionError = startEncoderPosition - traj.get((int) (elMili.time() - start)).get(2);
                // Set some cool pid values
                frontLeft.setVelocityPIDFCoefficients(RobotValues.KV, 0, 0, RobotValues.FEED_FORWARD * (traj.get((int) (elMili.time() - start + 5)).get(1)));
//                                         dashboardTelemetry.addData("frontLeft Speeed", frontLeft.getVelocity());
                dashboardTelemetry.addData("What it should be ", speed);
                dashboardTelemetry.addData("Velocity error", speed - frontLeft.getVelocity());
                dashboardTelemetry.update();

                speed = traj.get((int) (elMili.time() - start)).get(1);
                telemetry.addData("velocity", speed);
                telemetry.update();
                //               frontRight.setVelocity(speed);
                frontLeft.setVelocity(-speed);
//                    backRight.setVelocity(speed);
//                    backLeft.setVelocity(-speed);
            }
        }
    }

}

