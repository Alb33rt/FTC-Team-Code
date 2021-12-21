package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "ManualOp", group = "OpMode")
public class ManualOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor leftFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightFront = null;
    public DcMotor rightRear = null;

    public DcMotor Lift1;

    double chassisPower;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightRear = hardwareMap.get(DcMotor.class, "rightDrive");

        // Getting the Lift Motor
        Lift1 = hardwareMap.get(DcMotor.class, "lift");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        dashboard = FtcDashboard.getInstance();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            MecanumDrive(gamepad1);
            LiftDrive(gamepad2, Lift1);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    public void MecanumDrive(Gamepad gamepad) {
        double leftFrontPower, rightFrontPower, leftRearPower, rightRearPower;
        double x = gamepad.left_stick_x;
        double y = gamepad.left_stick_y;
        double turn  =  gamepad.right_stick_x;

        leftFrontPower = Range.clip(y - x - turn, -1.0, 1.0) ;
        rightFrontPower = Range.clip(y + x - turn, -1.0, 1.0) ;
        leftRearPower = Range.clip(y + x - turn, -1.0, 1.0);
        rightRearPower = Range.clip(y - x - turn, -1.0, 1.0);

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);
        telemetry.addData("Motors", "leftFront (%.2f), rightFront (%.2f), leftRear (%.2f), rightRear (%.2f)", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
    }

    public int LiftDrive(Gamepad gamepad, DcMotor liftMotor) {
        int targetPosition = 0;
        if (gamepad.dpad_up) {
            targetPosition += 3;
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setTargetPosition(targetPosition);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(1);
        }
        if (gamepad.dpad_down) {
            targetPosition -= 3;
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setTargetPosition(targetPosition);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(-1);
        }
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Adjusts the motor to the target position, which eliminates potential errors such as gravity.
        liftMotor.setPower(PIDController(targetPosition, liftMotor));
    }

    // PIDController takes in two parameters, the postition the motor is going to go to, and the motor.
    public double PIDController(double targetPosition, DcMotor motor) {
        double P = 1;
        double positionDifference = targetPosition - motor.getCurrentPosition();
        double power = positionDifference * P;
        return power;
    }

    public double LifterSpinDrive(Servo spinnerServo) {

        
    }
}
