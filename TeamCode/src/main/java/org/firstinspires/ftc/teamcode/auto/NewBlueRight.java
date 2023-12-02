/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class NewBlueRight extends LinearOpMode {

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

    private final int RIGHT_TURN = 300; // THIS WILL HAVE TO CHANGE, Also note this means a 90 degree turn, not turning right
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
    private DistanceSensor distance;

    private  int MAX_DISTANCE = 160;
    @Override
    public void runOpMode() {

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
        distance = hardwareMap.get(DistanceSensor.class, "distance");

        leftClaw.setDirection(Servo.Direction.REVERSE);


        //Arm settings
        armMotorPower = 0.15;
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
        LiftRight.setDirection(DcMotorSimple.Direction.FORWARD);
        LiftRight.setTargetPosition(0);
        LiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftLeft.setDirection(DcMotorSimple.Direction.REVERSE );
        LiftLeft.setTargetPosition(0);
        LiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LiftRight.setPower(0.8);
        LiftLeft.setPower(0.8);

        //initialize terminal

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* Start */
        waitForStart();

        sleep(1000);
        if(distance.getDistance(DistanceUnit.CM) < 140) {// FIRST CASE: If prop is middle

            forwardDrive(800);
            rotator.setPosition(CLAWDOWN);
            sleep(2000);
            rightClaw.setPosition(RIGHTOPEN);
            sleep(2000);
            rotator.setPosition(CLAWUP);
            sleep(2000);
//            leftTurn(RIGHT_TURN); // 90 degree turn
//            backwardsDrive(1200);
//
//            arm.setTargetPosition(ARMUP);
//            LiftRight.setTargetPosition(HANGINGUP);
//            LiftRight.setTargetPosition(HANGINGDOWN);
//            leftClaw.setPosition(LEFTOPEN);
//
//            rightShift(400);

        }else{
            rightShift(400);
            sleep(1000);
            if(distance.getDistance(DistanceUnit.CM) < 140) { // SECOND CASE: If prop is left
                forwardDrive(600);
                rotator.setPosition(CLAWDOWN);
                sleep(2000);
                rightClaw.setPosition(RIGHTOPEN);
                sleep(2000);
                rotator.setPosition(CLAWUP);
                sleep(2000);
//                leftTurn(200); // 90 degree turn
//                backwardsDrive(1200);
//                // Add left/right shift code
//                arm.setTargetPosition(ARMUP);
//                LiftRight.setTargetPosition(HANGINGUP);
//                LiftRight.setTargetPosition(HANGINGDOWN);
//                leftClaw.setPosition(LEFTOPEN);
//                rightShift(400);
            }else{ // THIRD CASE: Prop is right

                forwardDrive(200);
                leftTurn(RIGHT_TURN);
                forwardDrive(600);
                rotator.setPosition(CLAWDOWN);
                rightClaw.setPosition(RIGHTOPEN);
                rotator.setPosition(CLAWUP);

//                backwardsDrive(1600);
//                // Add left/right shift code
//                arm.setTargetPosition(ARMUP);
//                LiftRight.setTargetPosition(HANGINGUP);
//                LiftRight.setTargetPosition(HANGINGDOWN);
//                leftClaw.setPosition(LEFTOPEN);
//                rightShift(400);

            }


        }





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

