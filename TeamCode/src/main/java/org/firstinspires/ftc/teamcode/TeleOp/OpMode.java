package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class OpMode extends LinearOpMode {
    private final double DEAD_ZONE = 0.3;
    private final double RIGHT_OPEN = 0.16;
    private final double RIGHT_CLOSE = 0.70;
    private final double LEFT_OPEN = 0.17;
    private final double LEFT_CLOSE = 0.75;
    private final double CLAW_UP = 0.03;
    private final double CLAW_DOWN = CLAW_UP + 0.80;
    private final int ARM_UP = 1280;
    private final int ARM_DOWN = 0;
    private final int CLIMB = 6000;

    private boolean armUp = false;
    private boolean armDown = false;
    private boolean leftClose = false;
    private boolean leftOpen = false;
    private boolean rightClose = false;
    private boolean rightOpen = false;
    private boolean openBoth = false;
    private boolean closeBoth = false;
    private boolean clawUp = false;
    private boolean clawDown = false;
    private boolean liftInitiate = false;
    private boolean liftStart = false;
    private boolean canLift = false;
    private boolean launchPlane = false;
    private double leftStickYAxis = 0;
    private double leftStickXAxis = 0;
    private double rightStickXAxis = 0;
    private double leftFrontMotorPower = 0;
    private double leftBackMotorPower = 0;
    private double rightFrontMotorPower = 0;
    private double rightBackMotorPower = 0;
    private double armMotorPower = 0;
    private double planePosition = 0;
    private double rightPosition = RIGHT_CLOSE;
    private double leftPosition = LEFT_CLOSE;
    private double rotatorPosition = CLAW_UP;
    private int targetArmValue = ARM_DOWN;
    private int hangingnPosition = 0;
    private DcMotor motor;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor liftLeft = null;
    private DcMotor liftRight = null;
    private DcMotor arm = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private Servo rotator = null;
    private Servo plane = null;

    @Override
    public void runOpMode() {
        //declare hardware
        leftFrontDrive = hardwareMap.get(DcMotor.class, "driveMotorFour");
        leftBackDrive = hardwareMap.get(DcMotor.class, "driveMotorOne");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "driveMotorThree");
        rightBackDrive = hardwareMap.get(DcMotor.class, "driveMotorTwo");
        arm = hardwareMap.get(DcMotor.class, "armMotorOne");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rotator = hardwareMap.get(Servo.class, "rotator");
        liftLeft = hardwareMap.get(DcMotor.class, "LiftLeft");
        liftRight = hardwareMap.get(DcMotor.class, "LiftRight");
        plane = hardwareMap.get(Servo.class, "plane");

        leftClaw.setDirection(Servo.Direction.REVERSE);

        //Arm settings
        armMotorPower = 0.25;
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setTargetPosition(0);
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

        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        liftRight.setTargetPosition(0);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setDirection(DcMotorSimple.Direction.FORWARD );
        liftLeft.setTargetPosition(0);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftRight.setPower(0.8);
        liftLeft.setPower(0.8);

        planePosition = 0.0;
        hangingnPosition = 0;

        //initialize terminal
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            armUp = this.gamepad1.dpad_up;
            armDown = this.gamepad1.dpad_down;
            leftClose = this.gamepad1.left_trigger > 0.25;
            leftOpen = this.gamepad1.left_bumper;
            rightClose = this.gamepad1.right_trigger > 0.25;
            rightOpen = this.gamepad1.right_bumper;
            openBoth = this.gamepad1.b;
            closeBoth = this.gamepad1.x;
            leftStickYAxis = -this.gamepad1.left_stick_y * 0.8;
            leftStickXAxis = this.gamepad1.left_stick_x;
            rightStickXAxis = -this.gamepad1.right_stick_x;
            clawDown = this.gamepad1.a;
            clawUp = this.gamepad1.y;
            liftInitiate = this.gamepad1.back;
            liftStart = this.gamepad1.start;
            launchPlane = this.gamepad1.right_stick_button;

            // Movement code
            if (Math.abs(leftStickYAxis) >= DEAD_ZONE) {
                leftFrontMotorPower = leftStickYAxis;
                rightFrontMotorPower = leftStickYAxis;
                leftBackMotorPower = leftStickYAxis;
                rightBackMotorPower = leftStickYAxis;
            } else if (Math.abs(leftStickXAxis) >= DEAD_ZONE) {
                leftFrontMotorPower = leftStickXAxis;
                rightFrontMotorPower = -leftStickXAxis;
                leftBackMotorPower = leftStickXAxis;
                rightBackMotorPower = -leftStickXAxis;
            } else if (Math.abs(rightStickXAxis) >= DEAD_ZONE) {
                leftFrontMotorPower = rightStickXAxis;
                rightFrontMotorPower = -rightStickXAxis;
                leftBackMotorPower = -rightStickXAxis;
                rightBackMotorPower = rightStickXAxis;
            } else {
                leftFrontMotorPower = 0;
                rightFrontMotorPower = 0;
                leftBackMotorPower = 0;
                rightBackMotorPower = 0;
            }

            //arm code
            if (armUp) {
                liftRight.setPower(1.0);
                liftLeft.setPower(1.0);
                targetArmValue = ARM_UP;
                rotatorPosition = CLAW_UP;

            } else if (armDown) {
                liftRight.setPower(0.9);
                liftLeft.setPower(0.9);
                targetArmValue = ARM_DOWN;
            }

            //servo code
            if (clawUp) {
                rotatorPosition = CLAW_UP;
            } else if (clawDown) {
                rotatorPosition = CLAW_DOWN;
            }
            if (leftOpen || openBoth) {
                leftPosition = LEFT_OPEN;
            } else if (leftClose || closeBoth) {
                leftPosition = LEFT_CLOSE;
            }
            if (rightOpen || openBoth) {
                rightPosition = RIGHT_OPEN;
            } else if (rightClose || closeBoth) {
                rightPosition = RIGHT_CLOSE;
            }

            //Lift
            if (liftInitiate) {
                targetArmValue = 950;
                canLift = true;
            }
            if (liftStart && canLift) {
                targetArmValue = 300;
                 hangingnPosition = CLIMB;
                 canLift = false;
            }

            //airplane
            if (launchPlane) {
                planePosition = 1.0;
            }

            // Set motor & servo power
            leftFrontDrive.setPower(leftFrontMotorPower);
            leftBackDrive.setPower(leftBackMotorPower);
            rightFrontDrive.setPower(rightFrontMotorPower);
            rightBackDrive.setPower(rightBackMotorPower);
            arm.setTargetPosition(targetArmValue);
            arm.setPower(armMotorPower);
            leftClaw.setPosition(leftPosition);
            rightClaw.setPosition(rightPosition);
            rotator.setPosition(rotatorPosition);
            plane.setPosition(planePosition);
            liftLeft.setTargetPosition(hangingnPosition);
            liftRight.setTargetPosition(hangingnPosition);

            // Debug
            telemetry.addData("Status", "Running");
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.addData("Left Trigger", this.gamepad1.left_trigger);
            telemetry.addData("Left Bumper", this.gamepad1.left_bumper);
            telemetry.addData("Rotator Servo", rotator.getPosition());

            // Get arm position.
            //telemetry.addData("Tick position", "%4.2f", target);

            telemetry.update();
        }
    }

}
