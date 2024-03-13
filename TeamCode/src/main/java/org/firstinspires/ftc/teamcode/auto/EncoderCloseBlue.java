package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class EncoderCloseBlue extends LinearOpMode {
    private final double RIGHT_OPEN = 0.16;
    private final double RIGHT_CLOSE = 0.70;
    private final double LEFT_CLOSE = 0.75;
    private final double DRIVE_MOTOR_POWER = 0.35;
    private final double CLAW_UP = 0.03;
    private final double CLAW_DOWN = 0.79;
    private final double CLAW_HALF = 0.6 - 0.8/16;
    private final int ARM_BOARD_POSITION = 300 - 80;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor arm = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private Servo rotator = null;
    private ColorSensor colorRight = null, colorLeft = null;

    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "driveMotorFour");
        leftBackDrive = hardwareMap.get(DcMotor.class, "driveMotorOne");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "driveMotorThree");
        rightBackDrive = hardwareMap.get(DcMotor.class, "driveMotorTwo");
        arm = hardwareMap.get(DcMotor.class, "armMotorOne");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rotator = hardwareMap.get(Servo.class, "rotator");
        colorRight = hardwareMap.get(ColorSensor.class, "Color2");
        colorLeft = hardwareMap.get(ColorSensor.class, "Color1");

        //declare rotation direction
        leftClaw.setDirection(Servo.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        //set arm motor behavior
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.2);

        //set drive motor behavior
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setTargetPosition(0);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setPower(DRIVE_MOTOR_POWER);

        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setTargetPosition(0);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setPower(DRIVE_MOTOR_POWER);

        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setTargetPosition(0);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setPower(DRIVE_MOTOR_POWER);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setTargetPosition(0);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setPower(DRIVE_MOTOR_POWER);

        //set initial claw positions
        rotator.setPosition(CLAW_UP);
        rightClaw.setPosition(RIGHT_CLOSE);
        leftClaw.setPosition(LEFT_CLOSE);

        //telemetry initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        Forward(getTicksFromDistance(2.65));
        sleep(400);
        ShuffleRight(getTicksFromDistance(0.75));
        ShuffleRight(getTicksFromDistance(0.1));

        if (checkObject()) {
            //-------------------------------------------------right
            ShuffleLeft(getTicksFromDistance(0.8));
            PivotRight(getTicksFromDegree(90));
            Backward(getTicksFromDistance(0.3));
            rotator.setPosition(CLAW_DOWN);
            sleep(500);
            useClaw();
            PivotRight(getTicksFromDegree(180));
            arm.setTargetPosition(ARM_BOARD_POSITION);
            Forward(getTicksFromDistance(3.2));
            ShuffleRight(getTicksFromDistance(1.0));
            rotator.setPosition(CLAW_HALF);
            sleep(600);
            leftClaw.setPosition(RIGHT_OPEN);
            sleep(600);
            leftClaw.setPosition(RIGHT_CLOSE);
            rotator.setPosition(CLAW_UP);
            arm.setTargetPosition(0);
            Backward(getTicksFromDistance(0.3));

        } else {
            ShuffleLeft(getTicksFromDistance(1.4));
            ShuffleLeft(getTicksFromDistance(0.3));

            if (checkObject()) {
                //---------------------------------------------------------------left
                ShuffleRight(getTicksFromDistance(0.85));
                PivotLeft(getTicksFromDegree(90));
                rotator.setPosition(CLAW_DOWN);
                sleep(400);
                useClaw();
                sleep(200);
                Forward(getTicksFromDistance(0.3));
                ShuffleLeft(getTicksFromDistance(1.5));

                // Reset robot
                arm.setTargetPosition(ARM_BOARD_POSITION);
                rotator.setPosition(CLAW_HALF);
                Forward(getTicksFromDistance(3));
                ShuffleRight(getTicksFromDistance(1.3));
                Forward(getTicksFromDistance(0.3));
                leftClaw.setPosition(RIGHT_OPEN);
                sleep(400);
                leftClaw.setPosition(RIGHT_CLOSE);
                rotator.setPosition(CLAW_UP);
                arm.setTargetPosition(0);
                Backward(getTicksFromDistance(0.3));
            } else {
                //--------------------------------------------------------middle
                ShuffleRight(getTicksFromDistance(0.4));
                Backward(getTicksFromDistance(0.7));
                rotator.setPosition(CLAW_DOWN);
                sleep(600);
                Forward(getTicksFromDistance(0.2));
                useClaw();
                Backward(getTicksFromDistance(0.5));
                PivotLeft(getTicksFromDegree(90));
                arm.setTargetPosition(ARM_BOARD_POSITION);
                Forward(getTicksFromDistance(3.0));
                ShuffleRight(getTicksFromDistance(1));
                Forward(getTicksFromDistance(0.1));
                rotator.setPosition(CLAW_HALF);
                sleep(600);
                leftClaw.setPosition(RIGHT_OPEN);
                sleep(600);
                leftClaw.setPosition(RIGHT_CLOSE);
                rotator.setPosition(CLAW_UP);
                arm.setTargetPosition(0);
                Backward(getTicksFromDistance(0.3));
            }
        }
        telemetry.update();
    }

    private void useClaw() {
        rightClaw.setPosition(RIGHT_OPEN);
        sleep(600);
        rightClaw.setPosition(RIGHT_CLOSE);
        rotator.setPosition(CLAW_UP);
    }

    public int getTicksFromDistance(double inches){
        return (int) (inches * 480.0);
    }

    public int getTicksFromDegree(double degree){
        return (int) (degree/90*815);
    }

    public boolean checkObject(){
        int blue = 0, red = 0;

        for (int counter = 0; counter < 20; counter++) {
            if(colorRight.red() > 1200 || colorLeft.red() > 1200){
                red ++;
            } else if ((colorRight.blue() > 400 && colorRight.red() < colorRight.blue() / 2 + 100) ||
                    (colorLeft.blue() > 400 && colorLeft.red() < colorLeft.blue() / 2 + 100)) {
                blue ++;
            }
        }

        if (red >= 12 || blue >= 12) {
            return true;
        } else {
            return false;
        }
    }

    public void Forward(int distance){
        int lft = leftFrontDrive.getCurrentPosition() + distance;
        int lbt = leftBackDrive.getCurrentPosition() + distance;
        int rft = rightFrontDrive.getCurrentPosition() + distance;
        int rbt = rightBackDrive.getCurrentPosition() + distance;
        leftFrontDrive.setTargetPosition(lft);
        leftBackDrive.setTargetPosition(lbt);
        rightFrontDrive.setTargetPosition(rft);
        rightBackDrive.setTargetPosition(rbt);


        while(Math.abs(leftBackDrive.getCurrentPosition() - lbt) > 10 ||
                Math.abs(leftFrontDrive.getCurrentPosition() - lft) > 5 ||
                Math.abs(rightBackDrive.getCurrentPosition() - rbt) > 10 ||
                Math.abs(rightFrontDrive.getCurrentPosition() - rft) > 10) {
            sleep(10);
        }
        sleep(50);
    }

    public void Backward(int distance){
        int lft = leftFrontDrive.getCurrentPosition() - distance;
        int lbt = leftBackDrive.getCurrentPosition() - distance;
        int rft = rightFrontDrive.getCurrentPosition() - distance;
        int rbt = rightBackDrive.getCurrentPosition() - distance;
        leftFrontDrive.setTargetPosition(lft);
        leftBackDrive.setTargetPosition(lbt);
        rightFrontDrive.setTargetPosition(rft);
        rightBackDrive.setTargetPosition(rbt);


        while(Math.abs(leftBackDrive.getCurrentPosition() - lbt) > 10 ||
                Math.abs(leftFrontDrive.getCurrentPosition() - lft) > 5 ||
                Math.abs(rightBackDrive.getCurrentPosition() - rbt) > 10 ||
                Math.abs(rightFrontDrive.getCurrentPosition() - rft) > 10) {
            sleep(10);
        }
        sleep(50);
    }

    public void PivotRight(int distance){
        int lft = leftFrontDrive.getCurrentPosition() + distance;
        int lbt = leftBackDrive.getCurrentPosition() + distance;
        int rft = rightFrontDrive.getCurrentPosition() - distance;
        int rbt = rightBackDrive.getCurrentPosition() - distance;
        leftFrontDrive.setTargetPosition(lft);
        leftBackDrive.setTargetPosition(lbt);
        rightFrontDrive.setTargetPosition(rft);
        rightBackDrive.setTargetPosition(rbt);


        while(Math.abs(leftBackDrive.getCurrentPosition() - lbt) > 10 ||
                Math.abs(leftFrontDrive.getCurrentPosition() - lft) > 5 ||
                Math.abs(rightBackDrive.getCurrentPosition() - rbt) > 10 ||
                Math.abs(rightFrontDrive.getCurrentPosition() - rft) > 10) {
            sleep(10);
        }
        sleep(50);
    }

    public void PivotLeft(int distance){
        int lft = leftFrontDrive.getCurrentPosition() - distance;
        int lbt = leftBackDrive.getCurrentPosition() - distance;
        int rft = rightFrontDrive.getCurrentPosition() + distance;
        int rbt = rightBackDrive.getCurrentPosition() + distance;
        leftFrontDrive.setTargetPosition(lft);
        leftBackDrive.setTargetPosition(lbt);
        rightFrontDrive.setTargetPosition(rft);
        rightBackDrive.setTargetPosition(rbt);


        while(Math.abs(leftBackDrive.getCurrentPosition() - lbt) > 10 ||
                Math.abs(leftFrontDrive.getCurrentPosition() - lft) > 5 ||
                Math.abs(rightBackDrive.getCurrentPosition() - rbt) > 10 ||
                Math.abs(rightFrontDrive.getCurrentPosition() - rft) > 10) {
            sleep(10);
        }
        sleep(50);
    }

    public void ShuffleLeft(int distance){
        int lft = leftFrontDrive.getCurrentPosition() - distance;
        int lbt = leftBackDrive.getCurrentPosition() + distance;
        int rft = rightFrontDrive.getCurrentPosition() + distance;
        int rbt = rightBackDrive.getCurrentPosition() - distance;
        leftFrontDrive.setTargetPosition(lft);
        leftBackDrive.setTargetPosition(lbt);
        rightFrontDrive.setTargetPosition(rft);
        rightBackDrive.setTargetPosition(rbt);


        while(Math.abs(leftBackDrive.getCurrentPosition() - lbt) > 10 ||
                Math.abs(leftFrontDrive.getCurrentPosition() - lft) > 5 ||
                Math.abs(rightBackDrive.getCurrentPosition() - rbt) > 10 ||
                Math.abs(rightFrontDrive.getCurrentPosition() - rft) > 10) {
            sleep(10);
        }
        sleep(50);
    }

    public void ShuffleRight(int distance){
        int lft = leftFrontDrive.getCurrentPosition() + distance;
        int lbt = leftBackDrive.getCurrentPosition() - distance;
        int rft = rightFrontDrive.getCurrentPosition() - distance;
        int rbt = rightBackDrive.getCurrentPosition() + distance;
        leftFrontDrive.setTargetPosition(lft);
        leftBackDrive.setTargetPosition(lbt);
        rightFrontDrive.setTargetPosition(rft);
        rightBackDrive.setTargetPosition(rbt);


        while(Math.abs(leftBackDrive.getCurrentPosition() - lbt) > 10 ||
                Math.abs(leftFrontDrive.getCurrentPosition() - lft) > 5 ||
                Math.abs(rightBackDrive.getCurrentPosition() - rbt) > 10 ||
                Math.abs(rightFrontDrive.getCurrentPosition() - rft) > 10) {
            sleep(10);
        }
        sleep(50);
    }
}
