package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class OpMode extends LinearOpMode {
    private final double DEAD_ZONE = 0.2;

    private double leftFrontMotorPower = 0;
    private double leftBackMotorPower = 0;
    private double rightFrontMotorPower = 0;
    private double rightBackMotorPower = 0;
    private double armMotorPower = 0;
    private int targetTickValue = -1440;


    // Set motors
    private DcMotor motor;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor arm = null;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "driveMotorFour");
        leftBackDrive = hardwareMap.get(DcMotor.class, "driveMotorOne");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "driveMotorThree");
        rightBackDrive = hardwareMap.get(DcMotor.class, "driveMotorTwo");
        arm = hardwareMap.get(DcMotor.class, "armMotorOne");

        //Arm settings
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setTargetPosition(targetTickValue);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Drive motor settings
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean DPadUp = this.gamepad1.dpad_up;
            boolean DPadDown = this.gamepad1.dpad_down;

            double leftStickYAxis = -this.gamepad1.left_stick_y;
            double leftStickXAxis = this.gamepad1.left_stick_x;
            double rightStickXAxis = -this.gamepad1.right_stick_x;

            // Movement code
            if (Math.abs(leftStickYAxis) >= DEAD_ZONE) {
                leftFrontMotorPower = leftStickYAxis;
                rightFrontMotorPower = leftStickYAxis;
                leftBackMotorPower = leftStickYAxis;
                rightBackMotorPower = leftStickYAxis;
            } else if (Math.abs(leftStickXAxis) >= DEAD_ZONE) {
                leftFrontMotorPower = -leftStickXAxis;
                rightFrontMotorPower = leftStickXAxis;
                leftBackMotorPower = leftStickXAxis;
                rightBackMotorPower = -leftStickXAxis;
            } else if (Math.abs(rightStickXAxis) >= DEAD_ZONE) {
                leftFrontMotorPower = -rightStickXAxis;
                rightFrontMotorPower = rightStickXAxis;
                leftBackMotorPower = -rightStickXAxis;
                rightBackMotorPower = rightStickXAxis;
            } else if (DPadUp) {
                armMotorPower = 1;
                targetTickValue = -1440;
            } else if (DPadDown) {
                armMotorPower = 1;
                targetTickValue = 0;
            } else {
                leftFrontMotorPower = 0;
                rightFrontMotorPower = 0;
                leftBackMotorPower = 0;
                rightBackMotorPower = 0;
                armMotorPower = 0;
            }

            // Set motor power
            leftFrontDrive.setPower(leftFrontMotorPower);
            leftBackDrive.setPower(leftBackMotorPower);
            rightFrontDrive.setPower(rightFrontMotorPower);
            rightBackDrive.setPower(rightBackMotorPower);

            arm.setTargetPosition(targetTickValue);
            arm.setPower(armMotorPower);

            // Debug
            telemetry.addData("Status", "Running");

            // Get stick movement
            /*
            telemetry.addData("Left Stick Y Axis Number", "%4.2f", leftStickYAxis);
            telemetry.addData("Left Stick X Axis Number", "%4.2f", leftStickXAxis);
            telemetry.addData("Right Stick X Axis Number", "%4.2f", rightStickXAxis);
            */

            // Get motor power
            telemetry.addData("Left Motor Power", "%4.2f %4.2f", leftFrontMotorPower, leftBackMotorPower);
            telemetry.addData("Right Motor Power", "%4.2f %4.2f", rightFrontMotorPower, rightBackMotorPower);

            // Get arm position
            //telemetry.addData("Tick position", "%4.2f", target);

            telemetry.update();
        }
    }
}
