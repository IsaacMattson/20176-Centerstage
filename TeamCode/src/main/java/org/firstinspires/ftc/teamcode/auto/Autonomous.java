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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Scalar;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode {
    private final double RIGHT_OPEN = 0.16;
    private final double RIGHT_CLOSE = 0.70;
    private final double LEFT_OPEN = 0.17;
    private final double LEFT_CLOSE = 0.75;
    private final double CLAW_UP = 0.03;
    private final double CLAW_DOWN = CLAW_UP + 0.80;
    private final double MOTOR_POWER = 0.4;
    private final int ARM_UP = 1280;
    private final int ARM_DOWN = 0;

    private boolean armDown = false;
    private boolean leftClose = false;
    private boolean leftOpen = false;
    private boolean rightClose = false;
    private boolean rightOpen = false;
    private boolean openBoth = false;
    private boolean closeBoth = false;
    private boolean clawUp = false;
    private boolean clawDown = false;
    private boolean checkFront = true;
    private boolean objectFound = false;
    private double armMotorPower = 0;
    private double teamPropMaxDistance = 60;
    private double rightPosition = RIGHT_CLOSE;
    private double leftPosition = LEFT_CLOSE;
    private double rotatorPosition = CLAW_UP;
    private double distance = 0;
    private DistanceSensor distanceSensor;
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

    @Override
    public void runOpMode() {
        int rightTurnsMade = 0;
        int leftTurnsMade = 0;

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
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");

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

        //initialize terminal
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* Start */
        waitForStart();

        while (opModeIsActive()) {
            rectangle_thresholder_pipeline location = new rectangle_thresholder_pipeline();

            telemetry.addData("Location", location.getLocation());
        }
    }

    public class rectangle_thresholder_pipeline extends OpenCvPipeline {
        private String location = "nothing";
        public Scalar lower = new Scalar(0, 0, 0);
        public Scalar upper = new Scalar(255, 255, 255);

        private Mat hsvMat = new Mat();
        private Mat binaryMat = new Mat();
        private Mat maskedInputMat = new Mat();

        private Point topLeft1 = new Point(10, 0), topLeft2 = new Point(10, 0);
        private Point bottomRight1 = new Point(40, 20), bottomRight2 = new Point(40, 20);

        public rectangle_thresholder_pipeline() {

        }

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvMat, lower, upper, binaryMat);

            double w1 = 0, w2 = 0;
            for (int i = (int) topLeft1.x; i <= bottomRight1.x; i++) {
                for (int j = (int) topLeft1.y; j <= bottomRight1.y; j++) {
                    if (binaryMat.get(i, j)[0] == 225) {
                        w1++;
                    }
                }
            }

            for (int i = (int) topLeft2.x; i <= bottomRight2.x; i++) {
                for (int j = (int) topLeft2.y; j <= bottomRight2.y; j++) {
                    if (binaryMat.get(i, j)[0] == 225) {
                        w2++;
                    }
                }
            }

            if (w1 > w2) {
                location = "1";
            } else if (w1 < w2) {
                location = "2";
            }

            return binaryMat;
        }

        public String getLocation() {
            return location;
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

    private void rightShift(int ms) {
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

    private void leftShift(int ms) {
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

