package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class AutoRedRight extends LinearOpMode {

    private final double DEAD_ZONE = 0.3;
    private final double RIGHTOPEN = 0.16;
    private final double RIGHTCLOSE = 0.70;
    private final double LEFTOPEN = 0.17;
    private final double LEFTCLOSE = 0.75;
    private final int ARMUP = 1280;
    private final int ARMDOWN = 0;
    private final double CLAWUP = 0.03;
    private final double CLAWDOWN = CLAWUP + 0.80;
    private final int CLIMB = 3000;
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
    private int hangingnPos = HANGINGDOWN;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor arm = null;
    private DcMotor extend = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private Servo rotator = null;



    private DistanceSensor distance;
    private double teamPropMaxDistance = 75;

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

        distance = hardwareMap.get(DistanceSensor.class, "distance");

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
        arm.setPower(0.15);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();
        rotator.setPosition(0.0);
        rightClaw.setPosition(RIGHTCLOSE);
        leftClaw.setPosition(LEFTCLOSE);
        rotator.setPosition(CLAWDOWN);

        if(distance.getDistance(DistanceUnit.CM) < 140){// If prop is middle

            forwardDrive(800);
            rightClaw.setPosition(RIGHTOPEN);
            leftTurn(200);
            backwardsDrive(1200);
            //TODO: Place orange pixels
        }


        // This assumes we start facing the scoreboard


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
