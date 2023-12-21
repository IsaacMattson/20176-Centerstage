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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Close extends LinearOpMode {
    private final double RIGHT_OPEN = 0.16;
    private final double RIGHT_CLOSE = 0.70;
    private final double LEFT_OPEN = 0.17;
    private final double LEFT_CLOSE = 0.75;
    private final double CLAW_UP = 0.03;
    private final double CLAW_DOWN = CLAW_UP + 0.80;
    private final double MOTOR_POWER = 0.40;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor arm = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private Servo rotator = null;
    private ColorSensor ColorR = null, ColorL = null;
    private IMU gyro = null;
    private boolean isOnRed = false;

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
        ColorR = hardwareMap.get(ColorSensor.class, "Color2");
        ColorL = hardwareMap.get(ColorSensor.class, "Color1");
        gyro = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parmeters= new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        gyro.initialize(parmeters);

        leftClaw.setDirection(Servo.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.2);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotator.setPosition(CLAW_UP);
        rightClaw.setPosition(RIGHT_CLOSE);
        leftClaw.setPosition(LEFT_CLOSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        gyro.resetYaw();

        waitForStart();

        //drive forward to middle
        forwardDrive(1450);
        sleep(500);

        //check for right side
        leftShift(600);
        sleep(300);
        rightTurn(0);
        backwardsDrive(150);

        //Detect
        if (checkObject()) {
            //team prop is on the left;
            rightShift(600);
            sleep(300);
            leftTurn(90);
            sleep(300);
            backwardsDrive(600);
            sleep(300);
            rotator.setPosition(CLAW_DOWN);
            sleep(1000);
            forwardDrive(750);
            sleep(300);
            rightClaw.setPosition(RIGHT_OPEN);
            sleep(1000);
            rightClaw.setPosition(RIGHT_CLOSE);
            rotator.setPosition(CLAW_UP);
            sleep(1000);

            if (isOnRed) {
                backwardsDrive(1800);
                sleep(300);
                rightShift(200);
            } else {
                backwardsDrive(1800);
                sleep(300);
                leftShift(300);
            }

            sleep(1000);
            backwardsDrive(1800);
        } else {
            //check right
            rightTurn(0);
            sleep(300);
            forwardDrive(50);
            rightShift(600);
            sleep(300);
            rightShift(600);
            sleep(300);
            rightTurn(0);
            backwardsDrive(150);
            rightTurn(0);

            //Detect
            if (checkObject()) {
                //right
                leftTurn(0);
                leftShift(600);
                sleep(300);
                rightTurn(90);
                sleep(300);
                backwardsDrive(200);
                sleep(300);
                forwardDrive(200);
                rotator.setPosition(CLAW_DOWN);
                sleep(1000);
                rightClaw.setPosition(RIGHT_OPEN);
                sleep(1000);
                rightClaw.setPosition(RIGHT_CLOSE);
                rotator.setPosition(CLAW_UP);
                sleep(1000);

                if (isOnRed) {
                    leftTurn(90);
                    sleep(300);
                    leftShift(700);
                } else {
                    rightTurn(90);
                    sleep(300);
                    rightShift(700);
                }

                sleep(300);
                backwardsDrive(2000);
            } else {
                //Is at center
                forwardDrive(150);
                leftShift(600);
                backwardsDrive(650);
                rotator.setPosition(CLAW_DOWN);
                sleep(1000);
                forwardDrive(450);
                sleep(300);
                rightClaw.setPosition(RIGHT_OPEN);
                sleep(1000);
                rightClaw.setPosition(RIGHT_CLOSE);
                rotator.setPosition(CLAW_UP);
                sleep(1000);

                if (isOnRed) {
                    leftTurn(90);
                } else {
                    rightTurn(90);
                }

                sleep(300);
                backwardsDrive(1950);
            }
        }

        sleep(300);
        arm.setTargetPosition(1200);
        sleep(2500);
        leftClaw.setPosition(LEFT_OPEN);
        sleep(1000);
        arm.setTargetPosition(0);
        leftClaw.setPosition(LEFT_CLOSE);
        sleep(2500);
    }

    public boolean checkObject(){
        int blue = 0, red = 0;

        for (int counter = 0; counter < 50; counter++) {
            if(ColorR.red() > 1200 || ColorL.red() > 1200){
                red ++;
            } else if ((ColorR.blue() > 400 && ColorR.red() < ColorR.blue() / 2 + 100) ||
                    (ColorL.blue() > 400 && ColorL.red() < ColorL.blue() / 2 + 100)) {
                blue ++;
            }
        }

        if (red >= 20) {
            isOnRed = true;
            return true;
        } else if (blue >= 20) {
            return true;
        } else {
            return false;
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
    private void rightTurn(int degree) { // Not sure what values to make negative, will test
        while(gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) >= -degree + 8){
            leftBackDrive.setPower(MOTOR_POWER);
            rightBackDrive.setPower(-MOTOR_POWER);
            leftFrontDrive.setPower(MOTOR_POWER);
            rightFrontDrive.setPower(-MOTOR_POWER);
        }
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void leftTurn(int degree) {
        while(gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) <= degree - 8){
            leftBackDrive.setPower(-MOTOR_POWER);
            rightBackDrive.setPower(MOTOR_POWER);
            leftFrontDrive.setPower(-MOTOR_POWER);
            rightFrontDrive.setPower(MOTOR_POWER);
        }
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

}

