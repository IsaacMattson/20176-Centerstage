package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DuoOpMode extends LinearOpMode {
    private final double RIGHT_OPEN = 0.16;
    private final double RIGHT_CLOSE = 0.70;
    private final double LEFT_OPEN = 0.17;
    private final double LEFT_CLOSE = 0.75;
    private final double CLAW_UP = 0.03;
    private final double CLAW_DOWN = 0.79;
    private final int ARM_UP = 1230;
    private final int ARM_DOWN = 0;
    private final int CLIMB = 6500;
    private final double CLAW_HALF = 0.6;
    private final int ARM_BOARD_POSITION = 300;
    private boolean canLift = false;
    private boolean canOpen = false;
    private double driveSpeed = 1.0;
    private double armMotorPower = 0;
    private double planePosition = 0;
    private double rightPosition = RIGHT_CLOSE;
    private double leftPosition = LEFT_CLOSE;
    private double rotatorPosition = CLAW_UP;
    private int targetArmValue = ARM_DOWN;
    private int hangingPosition;
    private final int slow = 1;
    private boolean armDisabled = false;
    private boolean armUp = false;
    private final boolean canAdjustArm = false;
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
    private final ColorSensor Color = null;
    private final ColorRangeSensor ColorRange = null;


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

        planePosition = 0.0;
        hangingPosition = 0;

        //initialize terminal
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean leftClose = this.gamepad1.left_trigger > 0.25;
            boolean leftOpen = this.gamepad1.left_bumper;
            boolean rightClose = this.gamepad1.right_trigger > 0.25;
            boolean rightOpen = this.gamepad1.right_bumper;
            boolean openBoth = this.gamepad1.b;
            boolean closeBoth = this.gamepad1.x;
            boolean clawDown = this.gamepad1.a;
            boolean clawUp = this.gamepad1.y;
            boolean liftInitiate = this.gamepad2.back;
            boolean liftStart = this.gamepad2.start;
            boolean launchPlane = this.gamepad2.right_stick_button;

            // Movement code
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontMotorPower = (y + x + rx) / denominator;
            double leftBackMotorPower = (y - x + rx) / denominator;
            double rightFrontMotorPower = (y - x - rx) / denominator;
            double rightBackMotorPower = (y + x - rx) / denominator;

            //arm code
            boolean armUp = this.gamepad2.dpad_up;
            boolean armDown = this.gamepad2.dpad_down;

            //preset positions:
            if (this.gamepad2.a) {//this is the preset down position
                this.armUp = false;
                targetArmValue = ARM_DOWN;
                rotatorPosition = CLAW_UP;
                closeBoth = true;
                this.driveSpeed = 1.0;
            } else if (this.gamepad2.y) {//this is the preset up position
                this.armUp = true;
                targetArmValue = ARM_UP;
                rotatorPosition = CLAW_UP;
                canOpen = true;
                this.driveSpeed = 0.50;
            } else if (this.gamepad2.x) {// this is the front-facing board position line 0
                this.armUp = false;
                targetArmValue = ARM_BOARD_POSITION - 80;
                rotatorPosition = CLAW_HALF - 0.8 / 16;
                canOpen = true;
                this.driveSpeed = 0.50;
            } else if (this.gamepad2.b) {//this is the front-facing board position line 1
                this.armUp = false;
                targetArmValue = ARM_BOARD_POSITION + 40;
                rotatorPosition = CLAW_HALF + 0.4 / 16.0;
                canOpen = true;
                this.driveSpeed = 0.50;
            }

            if (armUp) {
                this.armUp = true;
                targetArmValue = ARM_UP;
                rotatorPosition = CLAW_UP;
                canOpen = true;
                this.driveSpeed = 0.50;
            } else if (armDown) {
                clawUp = true;
                this.armUp = false;
                targetArmValue = ARM_DOWN;
                rotatorPosition = CLAW_UP;
                closeBoth = true;
                this.driveSpeed = 1.0;
            }

            //servo code
            if (clawUp) {
                rotatorPosition = CLAW_UP;
                closeBoth = true;
                canOpen = false;
            } else if (clawDown) {
                rotatorPosition = CLAW_DOWN;
                canOpen = true;
            }

            if (this.armUp) {
                if ((rightOpen || openBoth) && canOpen) {
                    leftPosition = LEFT_OPEN;
                } else if (rightClose || closeBoth) {
                    leftPosition = LEFT_CLOSE;
                }

                if ((leftOpen || openBoth) && canOpen) {
                    rightPosition = RIGHT_OPEN;
                } else if (leftClose || closeBoth) {
                    rightPosition = RIGHT_CLOSE;
                }
            } else {
                if ((leftOpen || openBoth) && canOpen) {
                    leftPosition = LEFT_OPEN;
                } else if (leftClose || closeBoth) {
                    leftPosition = LEFT_CLOSE;
                }

                if ((rightOpen || openBoth) && canOpen) {
                    rightPosition = RIGHT_OPEN;
                } else if (rightClose || closeBoth) {
                    rightPosition = RIGHT_CLOSE;
                }
            }

            //Lift
            if (liftInitiate) {
                targetArmValue = 950;
                canLift = true;
            }

            if (liftStart && canLift) {
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(0);
                armDisabled = true;
//                targetArmValue = 300;
                hangingPosition = CLIMB;
                canLift = false;
                liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
                liftRight.setTargetPosition(0);
                liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                liftLeft.setTargetPosition(0);
                liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                liftRight.setPower(0.6);
                liftLeft.setPower(0.6);
                liftLeft.setTargetPosition(hangingPosition);
                liftRight.setTargetPosition(hangingPosition);
            }

            //airplane
            if (launchPlane) {
                planePosition = 0.12;
            }

            //reset arm encoder
            if (this.gamepad2.left_stick_button && this.arm.getCurrentPosition() <= 50) {
                arm.setPower(0);
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sleep(200);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(armMotorPower);
                targetArmValue = ARM_DOWN;
            }


            // Set motor & servo power
            leftFrontDrive.setPower(leftFrontMotorPower * this.driveSpeed);
            leftBackDrive.setPower(leftBackMotorPower * this.driveSpeed);
            rightFrontDrive.setPower(rightFrontMotorPower * this.driveSpeed);
            rightBackDrive.setPower(rightBackMotorPower * this.driveSpeed);
            if (!armDisabled) {
                if (arm.getCurrentPosition() > 900) {
                    arm.setPower(0.15);
                } else {
                    arm.setPower(0.25);
                }
                arm.setTargetPosition(targetArmValue);
                arm.setPower(armMotorPower);
            }
            leftClaw.setPosition(leftPosition);
            rightClaw.setPosition(rightPosition);
            rotator.setPosition(rotatorPosition);
            plane.setPosition(planePosition);
            // Debug
//            telemetry.update();
            //stops and timings

        }
    }

}
