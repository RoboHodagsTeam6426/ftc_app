package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by bachmhun on 12/1/2016.
 */

@Autonomous(name = "TestBotTestAuto")
public class TestBotTestAutoOpMode extends LinearOpMode {

    private DcMotor motorLeftDrive;
    private DcMotor motorRightDrive;

    GyroSensor sensorGyro;
    ModernRoboticsI2cGyro gyro;

    double target = 90;
    double turnSpeed = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        motorLeftDrive = hardwareMap.dcMotor.get("motorLeftDrive");
        motorRightDrive = hardwareMap.dcMotor.get("motorRightDrive");

        motorRightDrive.setDirection(DcMotor.Direction.REVERSE);

        sensorGyro = hardwareMap.gyroSensor.get("sensorGyro");
        gyro = (ModernRoboticsI2cGyro) sensorGyro;
        int zAccumulated = gyro.getIntegratedZValue();

        gyro.calibrate();


        // make sure the gyro is calibrated.
        while (!isStopRequested() && gyro.isCalibrating()) {
            telemetry.addData(">", "Gyro Calibrating. Do Not move!");
            sleep(50);
            idle();
            telemetry.update();
        }

        //displays that it has finished calibrating
        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();
        //end of gyro

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("gyro", gyro.getIntegratedZValue());
            telemetry.update();
            Log.d("gyro","gyro: " + gyro.getIntegratedZValue());
            sleep(10);
        }


    }
}
