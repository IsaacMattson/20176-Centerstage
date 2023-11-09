package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class OpMode extends LinearOpMode {
    private final double DEAD_ZONE = 0.3;
    private double leftFrontMotorPower = 0;
    private double leftBackMotorPower = 0;
    private double rightFrontMotorPower = 0;
    private double rightBackMotorPower = 0;
    private double armMotorPower = 0;
    private int targetArmValue = 0;
    private double extendMotorPower = 0;
    private int targetExtendValue = 0;
    private double rightPos = 1.0;
    private int rightTick = 100;
    private double leftPos = 0.35;
    private int leftTick = 100;
    private double rotatorPos = 1;
    private boolean upCom = false, downCom = false;


    // Set motors
    private DcMotor motor;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor arm = null;
    private DcMotor extend = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private Servo rotator = null;



    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "driveMotorFour");
        leftBackDrive = hardwareMap.get(DcMotor.class, "driveMotorOne");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "driveMotorThree");
        rightBackDrive = hardwareMap.get(DcMotor.class, "driveMotorTwo");
        arm = hardwareMap.get(DcMotor.class, "armMotorOne");
        extend = hardwareMap.get(DcMotor.class, "armMotorTwo");

        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");

        rotator = hardwareMap.get(Servo.class, "rotator");

        //Arm settings

        armMotorPower = 0.2;
        extendMotorPower = 1;


        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setDirection(DcMotorSimple.Direction.REVERSE);
        extend.setTargetPosition(0);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Drive motor settings
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //

        telemetry.addData("Status", "Initialized");
        telemetry.update();



        waitForStart();

        while (opModeIsActive()) {
            boolean DPadUp = this.gamepad1.dpad_up;
            boolean DPadDown = this.gamepad1.dpad_down;

            boolean leftClose = this.gamepad1.left_trigger > 0.25;
            boolean leftOpen = this.gamepad1.left_bumper;

            boolean rightClose = this.gamepad1.right_trigger > 0.25;
            boolean rightOpen = this.gamepad1.right_bumper;

            boolean openBoth = this.gamepad1.dpad_left;
            boolean closeBoth = this.gamepad1.dpad_right;

            boolean x = this.gamepad1.x;
            boolean b = this.gamepad1.b;



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
            } else {
                leftFrontMotorPower = 0;
                rightFrontMotorPower = 0;
                leftBackMotorPower = 0;
                rightBackMotorPower = 0;
            }
            //arm code


            if (DPadUp) {
                targetArmValue = 1200;
                armMotorPower = 0.2;
                targetExtendValue = 1200;
                upCom = true;
            } else if (DPadDown) {
                targetExtendValue = 1200;
                armMotorPower = 0.1;
                targetArmValue = 0;
                downCom = true;
            }

            if(upCom && arm.getCurrentPosition() > 800){
                targetExtendValue = 4150;
                armMotorPower = 0.05;
                upCom = false;
            }
            if(downCom && arm.getCurrentPosition() < 800){
                targetExtendValue = 600;
                downCom = false;
            }





            //servo code
            if(extend.getCurrentPosition() > 3000){
                rotatorPos = 0.0;
            }else{
                rotatorPos = 1.0;
            }


            if(leftOpen){
                if(leftPos <= 0.45){
                    leftPos = 0.65;
                    leftTick = 100;
                }else if(leftTick < 0){
                    leftPos = 0.60;
                }
            }
            if(leftClose){
                leftPos = 0.45;
            }

            if(rightOpen){
                if(rightPos >= 0.9){
                    rightPos = 0.7;
                    rightTick = 100;
                }else if(rightTick < 0){
                    rightPos = 0.75;
                }
            }
            if(rightClose){
                rightPos = 0.9;
            }

            if(openBoth){
                rightPos = 0.7;
                leftPos = 0.65 ;
            }
            if(closeBoth){
                rightPos = 0.9;
                leftPos = 0.45;
            }

//            if(x){
//                rotatorPos = 1.0;
//            }if(b){
//                rotatorPos = 0.0;
//            }


            // Set motor power
            leftFrontDrive.setPower(leftFrontMotorPower);
            leftBackDrive.setPower(leftBackMotorPower);
            rightFrontDrive.setPower(rightFrontMotorPower);
            rightBackDrive.setPower(rightBackMotorPower);

            arm.setTargetPosition(targetArmValue);
            arm.setPower(armMotorPower);
            extend.setTargetPosition(targetExtendValue);
            extend.setPower(extendMotorPower);

            leftClaw.setPosition(leftPos);
            rightClaw.setPosition(rightPos);
            rotator.setPosition(rotatorPos);

            rightTick --; leftTick --;



            // Debug
            telemetry.addData("Status", "Running");
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.addData("Extend Position", extend.getCurrentPosition());
            telemetry.addData("Left Trigger", this.gamepad1.left_trigger);
            telemetry.addData("Left Bumper", this.gamepad1.left_bumper);


            // Get arm position
            //telemetry.addData("Tick position", "%4.2f", target);

            telemetry.update();
        }
    }

}
