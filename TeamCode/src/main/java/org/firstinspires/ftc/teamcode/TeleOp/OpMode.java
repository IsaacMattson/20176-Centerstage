package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class OpMode extends LinearOpMode {
    private final double DEAD_ZONE = 0.3;
    private final double RIGHTOPEN = 0.02;
    private final double RIGHTCLOSE = 0.25;
    private final double LEFTOPEN = 0.95;
    private final double LEFTCLOSE = 0.65;
    private final int EXTENDUP = 4000;
    private final int EXTENDDOWN = 0;
    private final int ARMUP = 1200;
    private final int ARMDOWN = 0;
    private final double CLAWUP = 0.0;
    private final double CLAWDOWN = 0.78;
    private boolean DPadUp = false;
    private boolean DPadDown = false;
    private boolean leftClose = false;
    private boolean leftOpen = false;
    private boolean rightClose = false;
    private boolean rightOpen = false;
    private boolean openBoth = false;
    private boolean closeBoth = false;
    private boolean clawUp = false;
    private boolean clawDown = false;
    private double leftStickYAxis = 0;
    private double leftStickXAxis = 0;
    private double rightStickXAxis = 0;
    private double leftFrontMotorPower = 0;
    private double leftBackMotorPower = 0;
    private double rightFrontMotorPower = 0;
    private double rightBackMotorPower = 0;
    private double armMotorPower = 0;
    private int targetArmValue = ARMDOWN;
    private double extendMotorPower = 0;
    private int targetExtendValue = EXTENDDOWN;
    private double rightPos = RIGHTCLOSE;
    private double leftPos = LEFTCLOSE;
    private double rotatorPos = CLAWUP;
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
        //declare hardware
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
        //initialize terminal

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            DPadUp = this.gamepad1.dpad_up;
            DPadDown = this.gamepad1.dpad_down;
            leftClose = this.gamepad1.left_trigger > 0.25;
            leftOpen = this.gamepad1.left_bumper;
            rightClose = this.gamepad1.right_trigger > 0.25;
            rightOpen = this.gamepad1.right_bumper;
            openBoth = this.gamepad1.x;
            closeBoth = this.gamepad1.b;
            leftStickYAxis = -this.gamepad1.left_stick_y;
            leftStickXAxis = this.gamepad1.left_stick_x;
            rightStickXAxis = -this.gamepad1.right_stick_x;
            clawDown = this.gamepad1.a;
            clawUp = this.gamepad1.y;



            // Movement code
            if (Math.abs(leftStickYAxis) >= DEAD_ZONE) {
                leftFrontMotorPower = leftStickYAxis;
                rightFrontMotorPower = leftStickYAxis;
                leftBackMotorPower = leftStickYAxis;
                rightBackMotorPower = leftStickYAxis;
            } else if (Math.abs(leftStickXAxis) >= DEAD_ZONE) {
                leftFrontMotorPower = -leftStickXAxis;
                rightFrontMotorPower = leftStickXAxis;
                leftBackMotorPower = -leftStickXAxis;
                rightBackMotorPower = leftStickXAxis;
            } else if (Math.abs(rightStickXAxis) >= DEAD_ZONE) {
                leftFrontMotorPower = -rightStickXAxis;
                rightFrontMotorPower = rightStickXAxis;
                leftBackMotorPower = -rightStickXAxis;
                rightBackMotorPower = -rightStickXAxis;
            } else {
                leftFrontMotorPower = 0;
                rightFrontMotorPower = 0;
                leftBackMotorPower = 0;
                rightBackMotorPower = 0;
            }
            //arm code
            if (DPadUp) {
                targetArmValue = ARMUP;
                armMotorPower = 0.2;
                upCom = true;
                rotatorPos = CLAWUP;
            } else if (DPadDown) {
                targetExtendValue = EXTENDDOWN +225;
                downCom = true;
            }
            if(upCom && arm.getCurrentPosition() > 1100){
                targetExtendValue = EXTENDUP;
                upCom = false;
            }
            if(downCom && extend.getCurrentPosition() < 800){
                armMotorPower = 0.1;
                targetArmValue = ARMDOWN;
                downCom = false;
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

            // Set motor & servo power
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

            // Debug
            telemetry.addData("Status", "Running");
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.addData("Extend Position", extend.getCurrentPosition());
            telemetry.addData("Left Trigger", this.gamepad1.left_trigger);
            telemetry.addData("Left Bumper", this.gamepad1.left_bumper);
            telemetry.addData("Rotator Servo", rotator.getPosition());


            // Get arm position
            //telemetry.addData("Tick position", "%4.2f", target);

            telemetry.update();
        }
    }

}
