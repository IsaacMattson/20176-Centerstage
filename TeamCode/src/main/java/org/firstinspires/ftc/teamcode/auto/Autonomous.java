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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode {
    private final double RIGHT_OPEN = 0.16;
    private final double RIGHT_CLOSE = 0.70;
    private final double LEFT_OPEN = 0.17;
    private final double LEFT_CLOSE = 0.75;
    private final double CLAW_UP = 0.03;
    private final double CLAW_DOWN = CLAW_UP + 0.80;

    private final double MOTOR_POWER = 0.4;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor arm = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private Servo rotator = null;
    private ColorSensor Color = null;

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
        Color = hardwareMap.get(ColorSensor.class, "Color2");

        leftClaw.setDirection(Servo.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotator.setPosition(CLAW_UP);
        rightClaw.setPosition(RIGHT_CLOSE);
        leftClaw.setPosition(LEFT_CLOSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        //drive forward to middle
        forwardDrive(1300);
        sleep(500);

        //check for right side
        rightShift(600);
        sleep(500);

        //start detection;
        int counter = 50;
        int blue = 0, red = 0;

        while(counter > 0) {
            counter --;
            if(Color.red()> 1200){
                red ++;
            }else if(Color.blue() > 400 && Color.red() < Color.blue() / 2 + 100){
                blue ++;
            }
        }

        //check results
        if(red > 20 || blue > 20){
            //team prop is on the right;
            rightClaw.setPosition(RIGHT_OPEN);
            sleep(1000);
            telemetry.addData("Here", 0);
            telemetry.update();
        }else{
            //check front
            leftShift(600);
            sleep(500);
            leftTurn(1000);
            sleep(500);
            rightShift(600);
            sleep(500);
            //start detection
            counter = 50;
            blue = 0; red = 0;
            while(counter > 0){
                counter --;
                if(Color.red()> 1200){
                    red ++;
                }else if(Color.blue() > 400 && Color.red() < Color.blue() / 2 + 100){
                    blue ++;
                }
            }
            //check results
            if(red > 20 || blue > 20){
                rightClaw.setPosition(RIGHT_OPEN);
                sleep(1000);
                telemetry.addData("Here", 0);
                telemetry.update();
            }else{
                //it is at left
                leftShift(600);
                sleep(500);
                forwardDrive(600);
                sleep(500);
                stop();
            }

        }


    }

    private void forwardDrive(int ms) {
        leftBackDrive.setPower(MOTOR_POWER);
        rightBackDrive.setPower(MOTOR_POWER);
        leftFrontDrive.setPower(MOTOR_POWER);
        rightFrontDrive.setPower(MOTOR_POWER);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void backwardsDrive(int ms){
        leftBackDrive.setPower(-MOTOR_POWER);
        rightBackDrive.setPower(-MOTOR_POWER);
        leftFrontDrive.setPower(-MOTOR_POWER);
        rightFrontDrive.setPower(-MOTOR_POWER);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void leftShift(int ms) {
        leftBackDrive.setPower(MOTOR_POWER);
        rightBackDrive.setPower(-MOTOR_POWER);
        leftFrontDrive.setPower(-MOTOR_POWER);
        rightFrontDrive.setPower(MOTOR_POWER);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void rightShift(int ms) {
        leftBackDrive.setPower(-MOTOR_POWER);
        rightBackDrive.setPower(MOTOR_POWER);
        leftFrontDrive.setPower(MOTOR_POWER);
        rightFrontDrive.setPower(-MOTOR_POWER);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void rightTurn(int ms) { // Not sure what values to make negative, will test
        leftBackDrive.setPower(MOTOR_POWER);
        rightBackDrive.setPower(-MOTOR_POWER);
        leftFrontDrive.setPower(MOTOR_POWER);
        rightFrontDrive.setPower(-MOTOR_POWER);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void leftTurn(int ms) {
        leftBackDrive.setPower(-MOTOR_POWER);
        rightBackDrive.setPower(MOTOR_POWER);
        leftFrontDrive.setPower(-MOTOR_POWER);
        rightFrontDrive.setPower(MOTOR_POWER);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

}

