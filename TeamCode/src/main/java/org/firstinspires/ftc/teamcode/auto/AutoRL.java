package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AutoRL extends LinearOpMode {

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
        telemetry.addData("Hello,","FTC!");

        // Declaring things, self explanatory
        leftFrontDrive = hardwareMap.get(DcMotor.class, "driveMotorFour");
        leftBackDrive = hardwareMap.get(DcMotor.class, "driveMotorOne");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "driveMotorThree");
        rightBackDrive = hardwareMap.get(DcMotor.class, "driveMotorTwo");
        arm = hardwareMap.get(DcMotor.class, "armMotorOne");
        extend = hardwareMap.get(DcMotor.class, "armMotorTwo");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rotator = hardwareMap.get(Servo.class, "rotator");

        // Wheel Behaviour
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Arm Things
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        rotator.setPosition(0.0);
        rightClaw.setPosition(0.25);
        leftClaw.setPosition(0.65);
        rightShift(300);
        backwardsDrive(3600);
        rightShift(1200);
        backwardsDrive(100);
        arm.setPower(0.2);
        arm.setTargetPosition(1400);
        sleep(3000);
        //TODO: ARM STUFF!!!
        rightClaw.setPosition(0.05);
        leftClaw.setPosition(0.95);
        sleep(1000);
        arm.setPower(0.1);
        arm.setTargetPosition(0);
        sleep(3000);
        leftShift(1000);


    }

    private void forwardDrive(int ms){
        leftBackDrive.setPower(0.4);
        rightBackDrive.setPower(0.4);
        leftFrontDrive.setPower(0.4);
        rightFrontDrive.setPower(0.4);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void backwardsDrive(int ms){
        leftBackDrive.setPower(-0.4);
        rightBackDrive.setPower(-0.4);
        leftFrontDrive.setPower(-0.4);
        rightFrontDrive.setPower(-0.4);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void rightShift(int ms){
        leftBackDrive.setPower(0.4);
        rightBackDrive.setPower(-0.4);
        leftFrontDrive.setPower(-0.4);
        rightFrontDrive.setPower(0.4);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void leftShift(int ms){
        leftBackDrive.setPower(-0.4);
        rightBackDrive.setPower(0.4);
        leftFrontDrive.setPower(0.4);
        rightFrontDrive.setPower(-0.4);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void rightTurn(int ms){ // Not sure what values to make negative, will test
        leftBackDrive.setPower(0.4);
        rightBackDrive.setPower(-0.4);
        leftFrontDrive.setPower(0.4);
        rightFrontDrive.setPower(-0.4);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void leftTurn(int ms){
        leftBackDrive.setPower(-0.4);
        rightBackDrive.setPower(0.4);
        leftFrontDrive.setPower(-0.4);
        rightFrontDrive.setPower(0.4);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

}
