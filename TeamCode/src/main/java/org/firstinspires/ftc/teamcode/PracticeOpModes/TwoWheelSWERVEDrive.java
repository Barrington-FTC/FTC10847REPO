package org.firstinspires.ftc.teamcode.PracticeOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@TeleOp(name="2w Swerve")
public class TwoWheelSWERVEDrive extends LinearOpMode {
    //drive train
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightBackDrive = null;

    private DcMotorEx leftFrontRotation = null;
    private DcMotorEx rightBackRotation = null;
    private double L = 330.525; //wheelbase (front-back distance)inches
    private double  W = 257.4; //trackwidth (left-right distance)
    private double R = Math.sqrt(Math.pow(L,2) + Math.pow(W,2));
    private double A;
    private double B;
    private double C;
    private double D;
    private double leftFrontPower;
    private double rightBackPower;

    private double ticksPerRevolution = 145.1;
    private double TURRET_TICKS_PER_RADIAN = (ticksPerRevolution * 2) / (2 * Math.PI);


    @Override
    public void runOpMode() {
        //drive train
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftFrontRotation = hardwareMap.get(DcMotorEx.class, "leftFrontRotation");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBackDrive");
        rightBackRotation = hardwareMap.get(DcMotorEx.class, "rightBackRotation");
        setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontRotation.setDirection(DcMotor.Direction.REVERSE);
        rightBackRotation.setDirection(DcMotor.Direction.FORWARD);

        leftFrontRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontRotation.setTargetPosition(0);
        rightBackRotation.setTargetPosition(0);
        leftFrontRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontRotation.setPower(1);
        rightBackRotation.setPower(1);






        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) { // Loop
            //update all sensor variable to make cycle times faster at the start


            // --------------------------- WHEELS --------------------------- //

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;  // forward
            double rot = gamepad1.right_stick_x; // rotation

// Swerve kinematics
            double A = x - rot * (L / R);
            double B = x + rot * (L / R);
            double C = y - rot * (W / R);
            double D = y + rot * (W / R);

            //0 degrees is the wheels straght forward
            double flAngle = Math.atan2(B, D);
            double brAngle = Math.atan2(A, C);

// Calculate Power
            double leftFrontPower = Math.sqrt(B*B + D*D);
            double rightBackPower = Math.sqrt(A*A + C*C);

            double maxPower = Math.max(leftFrontPower, rightBackPower);
            if (maxPower > 1.0) {
                leftFrontPower /= maxPower;
                rightBackPower /= maxPower;
            }

            leftFrontDrive.setPower(leftFrontPower);
            rightBackDrive.setPower(rightBackPower);

            leftFrontRotation.setTargetPosition((int)(flAngle * TURRET_TICKS_PER_RADIAN));
            rightBackRotation.setTargetPosition((int)(brAngle * TURRET_TICKS_PER_RADIAN));



            // --------------------------- TELEMETRY --------------------------- //
            // Show the elapsed game time and wheel power.
            telemetry.addData("flAngle", flAngle);
            telemetry.addData("brAngle", brAngle);
            telemetry.addData("br encoder", rightBackRotation.getCurrentPosition());
            telemetry.addData("fl encoder", leftFrontRotation.getCurrentPosition());
            telemetry.update();
        }
    }

    // Dedicated method for the PID loop
    private void setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }
}
