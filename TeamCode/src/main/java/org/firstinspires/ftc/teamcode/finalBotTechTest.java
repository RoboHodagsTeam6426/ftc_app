package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by bachmhun on 1/3/2017.
 */
@Disabled
@TeleOp(name = "fianlbotTechTest")
public class finalBotTechTest extends LinearOpMode {

    private DcMotor motorLeftDrive;
    private DcMotor motorRightDrive;

    @Override

    public void runOpMode() throws InterruptedException {

        motorLeftDrive = hardwareMap.dcMotor.get("motorLeftDrive");
        motorRightDrive = hardwareMap.dcMotor.get("motorRightDrive");

        motorRightDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            motorLeftDrive.setPower(-gamepad1.left_stick_y);
            motorRightDrive.setPower(-gamepad1.right_stick_y);
        }
    }
}
