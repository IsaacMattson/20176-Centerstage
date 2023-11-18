package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AutoBR extends LinearOpMode {

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

        // This assumes we start facing the scoreboard

        //TODO: ARM STUFF!!!

        // TODO: Code for moving arm and placing pixels


    }

    private void forwardDrive(int ms){
        leftBackDrive.setPower(1);
        rightBackDrive.setPower(1);
        leftFrontDrive.setPower(1);
        rightFrontDrive.setPower(1);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void backwardsDrive(int ms){
        leftBackDrive.setPower(-1);
        rightBackDrive.setPower(-1);
        leftFrontDrive.setPower(-1);
        rightFrontDrive.setPower(-1);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void rightShift(int ms){
        leftBackDrive.setPower(1);
        rightBackDrive.setPower(-1);
        leftFrontDrive.setPower(-1);
        rightFrontDrive.setPower(1);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void leftShift(int ms){
        leftBackDrive.setPower(-1);
        rightBackDrive.setPower(1);
        leftFrontDrive.setPower(1);
        rightFrontDrive.setPower(-1);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void rightTurn(int ms){ // Not sure what values to make negative, will test
        leftBackDrive.setPower(1);
        rightBackDrive.setPower(-1);
        leftFrontDrive.setPower(1);
        rightFrontDrive.setPower(-1);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void leftTurn(int ms){
        leftBackDrive.setPower(-1);
        rightBackDrive.setPower(1);
        leftFrontDrive.setPower(-1);
        rightFrontDrive.setPower(1);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

}
