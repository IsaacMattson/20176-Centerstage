package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class OpMode extends LinearOpMode {
    private final double DEAD_ZONE = 0.3;
    private final double RIGHTOPEN = 0.16;
    private final double RIGHTCLOSE = 0.70;
    private final double LEFTOPEN = 0.17;
    private final double LEFTCLOSE = 0.75;
    private final int ARMUP = 1280;
    private final int ARMDOWN = 0;
    private final double CLAWUP = 0.03;
    private final double CLAWDOWN = CLAWUP + 0.80;
    private final int CLIMB = 6000;
    private final int HANGINGUP = 7500;
    private final int HANGINGDOWN = 500;
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
    private boolean LiftInitiate = false;
    private boolean LiftStart = false;
    private boolean canLift = false;
    private boolean launchPlane = false;
//    private boolean bothClaw = false;
//    private boolean rotatorButton = false;
//    private boolean armButton = false;
    private double leftStickYAxis = 0;
    private double leftStickXAxis = 0;
    private double rightStickXAxis = 0;
    private double leftFrontMotorPower = 0;
    private double leftBackMotorPower = 0;
    private double rightFrontMotorPower = 0;
    private double rightBackMotorPower = 0;
    private double armMotorPower = 0;
    private int targetArmValue = ARMDOWN;
    private double rightPos = RIGHTCLOSE;
    private double leftPos = LEFTCLOSE;
    private double rotatorPos = CLAWUP;
    private int hangingnPos = 0;
    private double planePos = 0;
    private DcMotor motor;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor LiftLeft = null;
    private DcMotor LiftRight = null;
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
        LiftLeft = hardwareMap.get(DcMotor.class, "LiftLeft");
        LiftRight = hardwareMap.get(DcMotor.class, "LiftRight");
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

        LiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        LiftRight.setTargetPosition(0);
        LiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftLeft.setDirection(DcMotorSimple.Direction.FORWARD );
        LiftLeft.setTargetPosition(0);
        LiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LiftRight.setPower(0.8);
        LiftLeft.setPower(0.8);

        planePos = 0.0;
        hangingnPos = 0;

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
//            bothClaw = this.gamepad1.b;
//            rotatorButton = this.gamepad2.y;
//            armButton = this.gamepad1.dpad_up;
            openBoth = this.gamepad1.b;
            closeBoth = this.gamepad1.x;
            leftStickYAxis = -this.gamepad1.left_stick_y * 0.8;
            leftStickXAxis = this.gamepad1.left_stick_x;
            rightStickXAxis = -this.gamepad1.right_stick_x;
            clawDown = this.gamepad1.a;
            clawUp = this.gamepad1.y;
            LiftInitiate = this.gamepad1.back;
            LiftStart = this.gamepad1.start;
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
                LiftRight.setPower(1.0);
                LiftLeft.setPower(1.0);
                targetArmValue = ARMUP;
                rotatorPos = CLAWUP;

            } else if (armDown) {
                LiftRight.setPower(0.9);
                LiftLeft.setPower(0.9);
                targetArmValue = ARMDOWN;
            }
            //servo code
            if(clawUp){
                rotatorPos = CLAWUP;
            }else if(clawDown){
                rotatorPos = CLAWDOWN;
            }
            if(leftOpen || openBoth){
                leftPos = LEFTOPEN;
            }else if(leftClose || closeBoth){
                leftPos = LEFTCLOSE;
            }
            if(rightOpen || openBoth){
                rightPos = RIGHTOPEN;
            }else if(rightClose || closeBoth){
                rightPos = RIGHTCLOSE;
            }
            //Lift
            if(LiftInitiate){
                targetArmValue = 950;
                canLift = true;
            }
            if(LiftStart && canLift){
                targetArmValue = 300;
                 hangingnPos = CLIMB;
                 canLift = false;
            }
            //airplane
            if(launchPlane){
                planePos = 1.0;
            }

;
            // Set motor & servo power
            leftFrontDrive.setPower(leftFrontMotorPower);
            leftBackDrive.setPower(leftBackMotorPower);
            rightFrontDrive.setPower(rightFrontMotorPower);
            rightBackDrive.setPower(rightBackMotorPower);
            arm.setTargetPosition(targetArmValue);
            arm.setPower(armMotorPower);
            leftClaw.setPosition(leftPos);
            rightClaw.setPosition(rightPos);
            rotator.setPosition(rotatorPos);
            plane.setPosition(planePos);
            LiftLeft.setTargetPosition(hangingnPos);
            LiftRight.setTargetPosition(hangingnPos);

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
