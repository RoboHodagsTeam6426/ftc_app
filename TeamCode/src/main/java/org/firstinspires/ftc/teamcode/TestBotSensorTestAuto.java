package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//imports imports sdks for devices like motors or certain sensors so the program recognizes the commands for them

/**
 * Created by bachmhun on 12/13/2016.
 */
//sets what type of program and the name that will be displayed on the app
@TeleOp(name = "TestBotSensorTestAuto")
//this makes a class called v that is displayed in the android tab and extends another class with more imports and set up to help you.
public class TestBotSensorTestAuto extends LinearOpMode {

    //declare motor names in a private class
    private DcMotor motorLeftDrive;
    private DcMotor motorRightDrive;

    // color sensor name declaration
    ColorSensor colorSensor;

    //range sensor declaration
    ModernRoboticsI2cRangeSensor rangeSensor;

    //overrides earlier class so that it can exceptions which helps it be less finicky.
    @Override
    // creates a class v, that needs to say runOpMode and it throw unexpected answers so as not to freeze the program.
    public void runOpMode() throws InterruptedException {

        //drive
        //corresponds the name in coding to the name in the configuration on phone
        motorLeftDrive = hardwareMap.dcMotor.get("leftDriveMotor");
        motorRightDrive = hardwareMap.dcMotor.get("rightDriveMotor");
        //code name ^                  configuration name ^

        //reverse one motor so they drive the same direction
        motorRightDrive.setDirection(DcMotor.Direction.REVERSE);
        //end of drive

        //MR color sensor
        //sets up an array of hsv values
        float hsvValues[] = {0F,0F,0F};

        final float values[] = hsvValues;

        waitForStart();

        //corresponds the name in coding to the name in phone. Just the same as the one above for the motors
        //but uses a different name v for what the actual hardware is (motor, servo, sensor, etc.)
        colorSensor = hardwareMap.colorSensor.get("sensorColor");

        //sets a boolean for display on the phone and is set to the state of the led on the color
        //sensor when the program starts.
        boolean bLedOn = false;

        //turns the led off when program is started.
        colorSensor.enableLed(true);
        //end of color sensor

        //MR 3 axis integrated gyro
        ModernRoboticsI2cGyro gyro;   // Hardware Device Object
        int xVal, yVal, zVal = 0;     // Gyro rate Values
        int heading = 0;              // Gyro integrated heading
        int angleZ = 0;

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("sensorGyro");

        //displays green font while the gyro is calibrating
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        // make sure the gyro is calibrated.
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        //displays that it has finished calibrating
        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();
        //end of gyro

        //MR range sensor
        // get a reference to our compass
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensorRange");
        //end of range sensor

        waitForStart();

        while (opModeIsActive()) {
            //drive
            //sets the motors to drive at the percentage the sticks are pushed, is negated so that forward goes forward
            //if not negated it would be inverted controls.
            motorLeftDrive.setPower(-gamepad1.left_stick_y);
            motorRightDrive.setPower(-gamepad1.right_stick_y);
            //end of drive

            //color sensor
            //says if said button on said controller is pressed to do this
            /*if (gamepad1.y) {
                //turns led on with a value of true
                colorSensor.enableLed(true);
                //sets the boolean to true for display on the phone
                bLedOn = true;
            }
            //same as before but different button and turns off not on
            if (gamepad1.b) {
                colorSensor.enableLed(false);
                bLedOn = false;
            }*/

            //converts the rgb values to hsv for the [phone display.
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

            // the telemetry command displays stings on the phone.
            telemetry.addLine("Color Sensor");
            //this telemetry uses the boolean from earlier to say whether the the led is on (true) or off(false)
            //telemetry.addData("LED", bLedOn ? "On" : "Off");
            //the next telemetry's display values for the color and hue
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);



            /*if (colorSensor.blue() < 2 && colorSensor.red() < 2 && time < 1000) {
                motorLeftDrive.setPower(1);
                motorRightDrive.setPower(-1);
                Thread.sleep(500);
                motorLeftDrive.setPower(0);
                motorRightDrive.setPower(0);
            }*/
            //end of color sensor

            //3 axis integrated gyro
            //if x is pressed then it resets the heading and sets 0 to whatever heading you are currently facing
            if (gamepad1.x) {
                gyro.resetZAxisIntegrator();
            }

            // get the x, y, and z values (rate of change of angle).
            xVal = gyro.rawX();
            yVal = gyro.rawY();
            zVal = gyro.rawZ();

            // get the heading info.
            // the Modern Robotics' gyro sensor keeps
            // track of the current heading for the Z axis only.
            heading = gyro.getHeading();
            angleZ  = gyro.getIntegratedZValue();

            //displays all the green to phone
            telemetry.addLine("3 Axis Integrated Gyro");
            telemetry.addData(">", "Press X to reset Heading.");
            telemetry.addData("0", "Heading %03d", heading);
            telemetry.addData("1", "Int. Ang. %03d", angleZ);
            telemetry.addData("2", "X av. %03d", xVal);
            telemetry.addData("3", "Y av. %03d", yVal);
            telemetry.addData("4", "Z av. %03d", zVal);
            //end of gyro

            //range sensor
            //displays range sensor values to phone
            telemetry.addLine("Range Sensor");
            telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
            telemetry.addData("raw optical", rangeSensor.rawOptical());
            telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
            telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
            //end of range sensor


        }

    }
}
