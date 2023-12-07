package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.*;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.time.LocalDateTime;
import java.util.Arrays;

@Autonomous
public class DistanceSensorTest extends LinearOpMode {
    double distance = 0;
    private DistanceSensor distanceSensor;
    int counter = 50;
    double[] distances = new double[50];


    @Override
    public void runOpMode() throws InterruptedException {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        telemetry.addData("Status", "Initialized");
        waitForStart();
        double sum = 0;

        while(opModeIsActive()){
            counter --;
            distance = distanceSensor.getDistance(DistanceUnit.CM);

            distances[counter] = distance;

            if(counter == 0){
                counter = 50;
                Arrays.sort(distances);
                sum = 0;
                for(int i = 5; i < 45; i++){
                    sum += distances[i];
                }
                sum /= 40;

            }
            telemetry.addData("distance in cm avg:", sum);
            telemetry.addData("current distance:", distance);
            telemetry.addData("counter:", counter);
//            telemetry.addData("time:",LocalDateTime.now().toString());
            telemetry.update();

        }
    }
}
