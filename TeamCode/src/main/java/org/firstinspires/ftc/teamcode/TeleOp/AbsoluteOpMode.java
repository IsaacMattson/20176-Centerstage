package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class AbsoluteOpMode extends LinearOpMode {
    private final double RIGHT_OPEN = 0.16;
    private final double RIGHT_CLOSE = 0.70;
    private final double LEFT_OPEN = 0.17;
    private final double LEFT_CLOSE = 0.75;
    private final double CLAW_UP = 0.03;
    private final double CLAW_DOWN = CLAW_UP + 0.80;
    private final int ARM_UP = 1230;
    private final int ARM_DOWN = 0;
    private final int CLIMB = 6000;
    private boolean canLift = false;
    private double armMotorPower = 0;
    private double planePosition = 0;
    private double rightPosition = RIGHT_CLOSE;
    private double leftPosition = LEFT_CLOSE;
    private double rotatorPosition = CLAW_UP;
    private int targetArmValue = ARM_DOWN;
    private int hangingPosition = 0;
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
    private ColorSensor Color = null;
    private ColorRangeSensor ColorRange = null;

    private IMU imu = null;

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
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parmeters= new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parmeters);
        leftClaw.setDirection(Servo.Direction.REVERSE);

        //IMU

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
        hangingPosition = 0;

        //initialize terminal
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        imu.resetYaw();

        waitForStart();

        while (opModeIsActive()) {
            boolean armUp = this.gamepad1.dpad_up;
            boolean armDown = this.gamepad1.dpad_down;
            boolean leftClose = this.gamepad1.left_trigger > 0.25;
            boolean leftOpen = this.gamepad1.left_bumper;
            boolean rightClose = this.gamepad1.right_trigger > 0.25;
            boolean rightOpen = this.gamepad1.right_bumper;
            boolean openBoth = this.gamepad1.b;
            boolean closeBoth = this.gamepad1.x;
            boolean clawDown = this.gamepad1.a;
            boolean clawUp = this.gamepad1.y;
            boolean liftInitiate = this.gamepad1.back;
            boolean liftStart = this.gamepad1.start;
            boolean launchPlane = this.gamepad1.right_stick_button;

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // Movement code
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double leftFrontMotorPower = (rotY + rotX + rx) / denominator;
            double leftBackMotorPower = (rotY - rotX + rx) / denominator;
            double rightFrontMotorPower = (rotY - rotX - rx) / denominator;
            double rightBackMotorPower = (rotY + rotX - rx) / denominator;

            if(this.gamepad1.left_stick_button){
                imu.resetYaw();
            }
            //arm code
            if (armUp) {
                targetArmValue = ARM_UP;
                rotatorPosition = CLAW_UP;
            } else if (armDown) {
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
                hangingPosition = CLIMB;
                canLift = false;
            }
            //airplane
            if (launchPlane) {
                planePosition = 1.0;
            }
            // Set motor & servo power
            leftFrontDrive.setPower(leftFrontMotorPower * 0.8);
            leftBackDrive.setPower(leftBackMotorPower * 0.8);
            rightFrontDrive.setPower(rightFrontMotorPower * 0.8);
            rightBackDrive.setPower(rightBackMotorPower * 0.8);
            liftLeft.setTargetPosition(hangingPosition);
            liftRight.setTargetPosition(hangingPosition);
            arm.setTargetPosition(targetArmValue);
            arm.setPower(armMotorPower);
            leftClaw.setPosition(leftPosition);
            rightClaw.setPosition(rightPosition);
            rotator.setPosition(rotatorPosition);
            plane.setPosition(planePosition);
            // Debug
//            telemetry.update();
        }
    }

}
