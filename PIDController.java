package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "PIDController", group = "PID")
@Disabled
public class PIDController extends LinearOpMode {
    DcMotorEx Motor1;
    double integral = 0;
    double lastError1 = 0;

    public static PIDCoefficients testPID = new PIDCoefficients(-100,0,15);

    FtcDashboard dashboard;
    ElapsedTime Motor1_timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        Motor1 = (DcMotorEx) hardwareMap.dcMotor.get("lift");
        Motor1.setDirection(DcMotor.Direction.FORWARD);

        Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();


        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Velocity of Motor1: ", Motor1.getVelocity());
            telemetry.update();
        }
    }

    void moveshoot1(double target){
        double error = Motor1.getVelocity();

        error = Motor1.getVelocity() - target;
        double deltaVelocity = lastError1 - error;
        integral += deltaVelocity * Motor1_timer.time();
        double derivative = deltaVelocity / Motor1_timer.time();
        double P = testPID.p * error;
//        double I = testPID.i * integral;
//        double D = testPID.d * derivative;
        Motor1.setVelocity(-(target+P));

        Motor1_timer.reset();
        lastError1 = error;
    }

}