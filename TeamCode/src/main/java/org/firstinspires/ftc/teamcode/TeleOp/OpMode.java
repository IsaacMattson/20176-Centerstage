package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class OpMode extends LinearOpMode {
    private DcMotor motor;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "driveMotorFour");
        leftBackDrive = hardwareMap.get(DcMotor.class, "driveMotorOne");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "driveMotorThree");
        rightBackDrive = hardwareMap.get(DcMotor.class, "driveMotorTwo");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double leftStickYAxis = -this.gamepad1.left_stick_y;
            double leftStickXAxis = this.gamepad1.left_stick_x;
            double rightStickXAxis = -this.gamepad1.right_stick_x;

            double leftFrontMotorPower = 0;
            double leftBackMotorPower = 0;
            double rightFrontMotorPower = 0;
            double rightBackMotorPower = 0;

            if (Math.abs(leftStickYAxis) >= 0.1) {
                leftFrontMotorPower = leftStickYAxis;
                rightFrontMotorPower = leftStickYAxis;
                leftBackMotorPower = leftStickYAxis;
                rightBackMotorPower = leftStickYAxis;
            } else if (Math.abs(leftStickXAxis) >= 0.1) {
                leftFrontMotorPower = leftStickXAxis;
                rightFrontMotorPower = -leftStickXAxis;
                leftBackMotorPower = -leftStickXAxis;
                rightBackMotorPower = leftStickXAxis;
            } else if (Math.abs(rightStickXAxis) >= 0.1) {
                leftFrontMotorPower = rightStickXAxis;
                rightFrontMotorPower = -rightStickXAxis;
                leftBackMotorPower = rightStickXAxis;
                rightBackMotorPower = -rightStickXAxis;
            }

            leftFrontDrive.setPower(leftFrontMotorPower);
            leftBackDrive.setPower(leftBackMotorPower);
            rightFrontDrive.setPower(rightFrontMotorPower);
            rightBackDrive.setPower(rightBackMotorPower);

            telemetry.addData("Status", "Running");

            telemetry.addData("Left Stick Y Axis Number", "%4.2f", leftStickYAxis);
            telemetry.addData("Left Stick X Axis Number", "%4.2f", leftStickXAxis);
            telemetry.addData("Right Stick X Axis Number", "%4.2f", rightStickXAxis);

            telemetry.addData("Left Motor Power", "%4.2f %4.2f", leftFrontMotorPower, leftBackMotorPower);
            telemetry.addData("Right Motor Power", "%4.2f %4.2f", rightFrontMotorPower, rightBackMotorPower);

            telemetry.update();
        }
    }
}
