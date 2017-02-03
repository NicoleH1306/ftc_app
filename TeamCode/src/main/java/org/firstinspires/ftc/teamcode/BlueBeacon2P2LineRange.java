/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
  The code is written using a method called: encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches,
                             double leftBackInches, double rightBackInches)
  that performs the actual movement.

  This methods assumes that each movement is relative to the last stopping place.
*/

@Autonomous(name="BlueBeacon2P2LineRange", group="Official")
//Disabled
public class BlueBeacon2P2LineRange extends LinearOpMode {

    //private ElapsedTime runtime = new ElapsedTime();

    //Declaring the drivetrain motors
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;

    //Declares the hand motor
    //This is used in Autonmous to move them out of the way
    DcMotor elevatorMotor = null;

    //Declares the color sensor
    //Used for detecting which button to hit
    ColorSensor colorSensor;

    //Declares the optical distance sensor
    //Used for finding the line beneath the beacon
    OpticalDistanceSensor lineSensor;

    ModernRoboticsI2cRangeSensor rangeSensor;





    static final double     COUNTS_PER_MOTOR_REV    = 1100 ;    // 1100 encoder points per rotation
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No gear reduction because of direct drive
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);


    @Override
    public void runOpMode() throws InterruptedException {

        //------------------------------------------------------------------------------------------
        //Initialize the hardware variables.


        //Drivetrain motors
        leftFrontMotor = hardwareMap.dcMotor.get("left front motor");
        rightFrontMotor = hardwareMap.dcMotor.get("right front motor");
        leftBackMotor = hardwareMap.dcMotor.get("left back motor");
        rightBackMotor = hardwareMap.dcMotor.get("right back motor");

        //Hands motor
        elevatorMotor = hardwareMap.dcMotor.get("elevator motor");

        //The sensors
        colorSensor = hardwareMap.colorSensor.get("beacon color");
        lineSensor = hardwareMap.opticalDistanceSensor.get("ods");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");

        //Turn the led off on the colorsensor
        colorSensor.enableLed(false);

        //------------------------------------------------------------------------------------------
        //Set the drive motor directions:


        //Left and right motors set in different directions of miror effect
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        elevatorMotor.setDirection(DcMotor.Direction.FORWARD);
        //private ElapsedTime     runtime = new ElapsedTime();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        //------------------------------------------------------------------------------------------
        //Set the encoders

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        //Reset the encoders to zero
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        //Set the mode for encoder to run
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message of the current encoder postion
        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                rightBackMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(), leftFrontMotor.getCurrentPosition(), rightFrontMotor.getCurrentPosition());
        telemetry.update();


        //------------------------------------------------------------------------------------------
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Set the base value for the optical distance sensor
        double baseV = lineSensor.getRawLightDetected();


        //------------------------------------------------------------------------------------------
        //Getting to the first beacon

        elevatorMotor.setPower(-0.5);
        sleep(1500);
        elevatorMotor.setPower(0);


        encoderDrive(0.4, 27, 27, 27, 27); //Drive forward 27 inches
        sleep(200);
        encoderDrive(0.4, 14, -14, 14, -14); //Turn to face the beacon
        sleep(200);
        encoderDrive(0.4, 49, 49, 49, 49); //Drive forward 47 inches to the beacon
        sleep(200);
        encoderDrive(0.4, -14, 14, -14, 14); //Turn robot to be pararell with beacon
        sleep(200);

        //------------------------------------------------------------------------------------------
        //lining up with the beacon

        //Keep driving until change in the sensor value caused by the white line
        while (baseV + 1 > lineSensor.getRawLightDetected() && opModeIsActive()) {
            leftFrontMotor.setPower(0.1);
            rightFrontMotor.setPower(0.1);
            leftBackMotor.setPower(0.1);
            rightBackMotor.setPower(0.1);

            telemetry.addData("ods", "base at %7f", baseV);
            telemetry.addData("ods", "value at %7f", lineSensor.getRawLightDetected());
            telemetry.update();
        }

        //When robot alligned to beacon stop
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);

        sleep(200);


        //------------------------------------------------------------------------------------------
        //Activating the first beacon

        while(rangeSensor.getDistance(DistanceUnit.INCH) > 4 && opModeIsActive())
        {
            leftFrontMotor.setPower(0.1);
            rightFrontMotor.setPower(-0.1);
            leftBackMotor.setPower(-0.1);
            rightBackMotor.setPower(0.1);
        }

        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);




        //Reading color first beacon
        //If color is blue
        if (colorSensor.blue() > 2) {
            encoderDrive(0.3, 3, 3, 3, 3);            //Drive backwards 3 inches to align with button
            sleep(200);
            while(rangeSensor.getDistance(DistanceUnit.INCH) > 3 && opModeIsActive())
            {
                leftFrontMotor.setPower(0.1);
                rightFrontMotor.setPower(-0.1);
                leftBackMotor.setPower(-0.1);
                rightBackMotor.setPower(0.1);
            }

            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);

            encoderDrive(0.3, -4, 4, 4, -4);              //Back away from the beacon
            sleep(200);
            encoderDrive(0.4, 41, 41, 41, 41);            //Drive to the next beacon
            sleep(200);
        }
        //If color is red
        else if (colorSensor.red() > 2) {
            encoderDrive(0.3, -5, -5, -5, -5);                //Drive forward to align sensor to beacon
            sleep(200);
            if (colorSensor.blue() > 2)                         //Check to see if color is blue
            {
                encoderDrive(0.3, -4, -4, -4, -4);            //Drive forward to align with button
                sleep(200);
                while(rangeSensor.getDistance(DistanceUnit.INCH) > 3 && opModeIsActive())
                {
                    leftFrontMotor.setPower(0.1);
                    rightFrontMotor.setPower(-0.1);
                    leftBackMotor.setPower(-0.1);
                    rightBackMotor.setPower(0.1);
                }

                leftFrontMotor.setPower(0);
                rightFrontMotor.setPower(0);
                leftBackMotor.setPower(0);
                rightBackMotor.setPower(0);
                encoderDrive(0.3, -4, 4, 4, -4);          //Back away from the beacon
            }
            encoderDrive(0.4, 52, 52, 52, 52);    //Drive to second beacon
            sleep(200);

        }
        //Neither blue or red is read
        else {
            encoderDrive(0, 0, 0, 0, 0);                    //Stopping the drivetrain
        }




        //------------------------------------------------------------------------------------------
        //Activating the second Beacon

        //Keep driving until change in the sensor value caused by the white line
        while (baseV + 1 > lineSensor.getRawLightDetected() && opModeIsActive()) {
            leftFrontMotor.setPower(0.1);
            rightFrontMotor.setPower(0.1);
            leftBackMotor.setPower(0.1);
            rightBackMotor.setPower(0.1);

            telemetry.addData("ods", "base at %7f", baseV);
            telemetry.addData("ods", "value at %7f", lineSensor.getRawLightDetected());
            telemetry.update();
        }

        while(rangeSensor.getDistance(DistanceUnit.INCH) > 4 && opModeIsActive())
        {
            leftFrontMotor.setPower(0.1);
            rightFrontMotor.setPower(-0.1);
            leftBackMotor.setPower(-0.1);
            rightBackMotor.setPower(0.1);
        }

        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);

        if (colorSensor.blue() > 2) {
            encoderDrive(0.3, 3, 3, 3, 3);            //Drive backwards 3 inches to align with button
            sleep(200);
            while(rangeSensor.getDistance(DistanceUnit.INCH) > 3 && opModeIsActive())
            {
                leftFrontMotor.setPower(0.1);
                rightFrontMotor.setPower(-0.1);
                leftBackMotor.setPower(-0.1);
                rightBackMotor.setPower(0.1);
            }

            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);

            encoderDrive(0.3, -4, 4, 4, -4);              //Back away from the beacon
            sleep(200);

        }
        //If color is red
        else if (colorSensor.red() > 2) {
            encoderDrive(0.3, -5, -5, -5, -5);                //Drive forward to align sensor to beacon
            sleep(200);
            if (colorSensor.blue() > 2)                         //Check to see if color is blue
            {
                encoderDrive(0.3, -4, -4, -4, -4);            //Drive forward to align with button
                sleep(200);
                while(rangeSensor.getDistance(DistanceUnit.INCH) > 3 && opModeIsActive())
                {
                    leftFrontMotor.setPower(0.1);
                    rightFrontMotor.setPower(-0.1);
                    leftBackMotor.setPower(-0.1);
                    rightBackMotor.setPower(0.1);
                }

                leftFrontMotor.setPower(0);
                rightFrontMotor.setPower(0);
                leftBackMotor.setPower(0);
                rightBackMotor.setPower(0);
                encoderDrive(0.3, -4, 4, 4, -4);          //Back away from the beacon
            }


        }
        //Neither blue or red is read
        else {
            encoderDrive(0, 0, 0, 0, 0);                    //Stopping the drivetrain
        }




        telemetry.addData("Path", "Complete");                 //Saying that it is complete
        telemetry.update();
        sleep(3000);
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches,
                             double leftBackInches, double rightBackInches
                             /*double timeoutS*/) throws InterruptedException {


        //Keeps record of the wanted poistion of the encoder
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontMotor.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontMotor.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget =  leftBackMotor.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBackMotor.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

            //Set the target postion to the new target postion set above
            leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            rightFrontMotor.setTargetPosition(newRightFrontTarget);
            leftBackMotor.setTargetPosition(newLeftBackTarget);
            rightBackMotor.setTargetPosition(newRightBackTarget);

            // Change mode to run to position
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            //Set speed of the motors
            //This make the motors move and the encoders count
            leftFrontMotor.setPower(Math.abs(speed));
            rightFrontMotor.setPower(Math.abs(speed));
            leftBackMotor.setPower(Math.abs(speed));
            rightBackMotor.setPower(Math.abs(speed));



            // keep looping while we are still active, and there is time left, and all motors are running.
            while (opModeIsActive() &&
                   (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() && leftBackMotor.isBusy() && rightBackMotor.isBusy())) {

                // Display it for the driver.
                // Target poistion
                telemetry.addData("Path1",  "Running to %7d :%7d, :%7d :%7d", newLeftFrontTarget,  newRightFrontTarget, newLeftBackTarget, newRightBackTarget);

                // The current poistion
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                                            leftFrontMotor.getCurrentPosition(),
                                            rightFrontMotor.getCurrentPosition(),
                                            leftBackMotor.getCurrentPosition(),

                                            rightBackMotor.getCurrentPosition());
                //The current speed
                telemetry.addData("Power",  "Power at %7f :%7f %7f %7f",
                        leftFrontMotor.getPower(),
                        rightFrontMotor.getPower(),
                        leftBackMotor.getPower(),
                        rightBackMotor.getPower());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all drivetrain motors
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);

            // Reset mode to run
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
