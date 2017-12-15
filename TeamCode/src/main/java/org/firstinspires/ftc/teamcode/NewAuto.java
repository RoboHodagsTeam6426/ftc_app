package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Hunter on 2/4/2017.
 */
@Autonomous
public class NewAuto extends LinearOpMode{
    private DcMotor motorLeftDrive;
    private DcMotor motorRightDrive;

    public  void runOpMode() throws InterruptedException{
        motorLeftDrive = hardwareMap.dcMotor.get("motorLeftDrive");
        motorRightDrive = hardwareMap.dcMotor.get("motorRightDrive");

        motorRightDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        Thread.sleep(10000);
        motorLeftDrive.setPower(-1);
        motorRightDrive.setPower(-1);
        Thread.sleep(1200);
        motorLeftDrive.setPower(0);
        motorRightDrive.setPower(0);
    }
}
