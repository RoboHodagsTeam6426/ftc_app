package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by bachmhun on 1/5/2017.
 */
@Disabled
@Autonomous(name = "gyroDriveTest")
public class gyroDriveTest extends LinearOpMode {

    private DcMotor motorLeftDrive;
    private DcMotor motorRightDrive;


    public void runOpMode() throws InterruptedException {

        motorLeftDrive = hardwareMap.dcMotor.get("motorLeftDrive");
        motorRightDrive = hardwareMap.dcMotor.get("motorRightDrive");

        motorRightDrive.setDirection(DcMotor.Direction.REVERSE);

        ModernRoboticsI2cGyro gyro;

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("sensorGyro");

        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        waitForStart();



    }
}
