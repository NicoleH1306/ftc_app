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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/*
  This is a specific program designed for this robot.

  This is the main program for the robot and controls all parts of the
  robot. It takes in the input of two controllers. The first controller
  controls the drivetrain, collector, and shooter. The second controller
  controls the elevator and the hands.
 */

@TeleOp(name="main", group="official")
//@Disabled
public class Main extends LinearOpMode {

    //Drive train motors
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;

    //Collector motor
    DcMotor collector = null;

    //Shooter motor
    DcMotor shooter = null;

    //The elevator and hands motor for the lift
    DcMotor elevatorMotor = null;
    DcMotor handsMotor = null;


    //The servo that stops the particle from climbing the collector
    Servo stopper;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //------------------------------------------------------------------------------------------
        //Initialize the hardware variables.

        //Drive train motors
        leftFrontMotor  = hardwareMap.dcMotor.get("left front motor");
        rightFrontMotor = hardwareMap.dcMotor.get("right front motor");
        leftBackMotor = hardwareMap.dcMotor.get("left back motor");
        rightBackMotor = hardwareMap.dcMotor.get("right back motor");

        //Collector motor
        collector = hardwareMap.dcMotor.get("collector motor");

        //Shooter motor
        shooter = hardwareMap.dcMotor.get("shooter motor");

        //Motor for stopping the particle
        stopper = hardwareMap.servo.get("stopper");

        //Motors for the elevator and hands
        elevatorMotor = hardwareMap.dcMotor.get("elevator motor");
        handsMotor = hardwareMap.dcMotor.get("hands motor");

        //------------------------------------------------------------------------------------------
        //Set the motor directions:

        //Drivetrain motors
        //Left and right motors are set in different directions because of mirror effect
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        //Collector motor
        collector.setDirection(DcMotor.Direction.FORWARD);

        //Shooter motor
        shooter.setDirection((DcMotor.Direction.REVERSE));

        //------------------------------------------------------------------------------------------
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //------------------------------------------------------------------------------------------
        //Intalizes the varibles after the program has started running

        boolean collectorToggle = false;     //Starting and stopping of the collector
        int collectorSpeed = 0;              //Stores the speed of the collector

        int leftBState = 0;                  //Stores when the left bumper is pushed

        stopper.setPosition(0.1);            //Set the beginning postion of the servo


        //------------------------------------------------------------------------------------------
        // Runs until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            //--------------------------------------------------------------------------------------
            //Setting up the controls. Assigns the gamepad vaules to a varible

            //Drive Train setup
            double leftX = gamepad1.left_stick_x;
            double leftY = gamepad1.left_stick_y;
            double rightX = gamepad1.right_stick_x;

            //Collecotr setup
            boolean leftB = gamepad1.left_bumper;
            double leftTr = gamepad1.left_trigger;

            //Shooter setup
            double rightTr = gamepad1.right_trigger;

            //Elevator setup
            double elevator = gamepad2.left_stick_y;
            double hands = gamepad2.right_stick_y;


            //Equation for all direction use of mechanum wheels converted to a varible
            double LFM = leftY - leftX - rightX;
            double RFM = leftY + leftX + rightX;
            double LBM = leftY + leftX - rightX;
            double RBM = leftY - leftX + rightX;

            //Creates a smaller curve for more control over the motors
            LFM = Math.pow(LFM, 2) * Math.signum(LFM);
            RFM = Math.pow(RFM, 2) * Math.signum(RFM);
            LBM = Math.pow(LBM, 2) * Math.signum(LBM);
            RBM = Math.pow(RBM, 2) * Math.signum(RBM);

            //Setting the power level for the drivetrain motors
            leftFrontMotor.setPower(LFM);
            rightFrontMotor.setPower(RFM);
            leftBackMotor.setPower(LBM);
            rightBackMotor.setPower(RBM);



            //--------------------------------------------------------------------------------------
            //Code for the collector

            //Collecotr toggles between on and off on the release of the bumper
            if(leftB)                                //If bumper is pressed
            {
                leftBState = 1;                      //Record that button has been pressed


            }
            else                                   //When the bumper is not pressed
            {
                if(leftBState == 1)                //If the button has been pressed
                {
                    //Tells whether to turn on or off
                    //If currently running turn off
                    if(collectorToggle)
                    {
                        collectorSpeed = 0;
                        collectorToggle = false;
                    }
                    //If currently off turn it on
                    else if(!collectorToggle)
                    {
                        collectorSpeed = 1;
                        collectorToggle = true;
                    }

                    //Resetting the button press
                    leftBState = 0;
                }


            }


            //Reverses the direction of the collector
            if(leftTr >= 0.8)
            {
                collectorSpeed = -Math.abs(collectorSpeed);
            }
            else
            {
                collectorSpeed = Math.abs(collectorSpeed);
            }


            //Set the power of the motor
            collector.setPower(collectorSpeed);



            //--------------------------------------------------------------------------------------
            //Code for shooter

            //If the right trigger button is pressed
            if(rightTr >= 0.8)
            {
                //Stops all the robot from moving during shooting
                leftFrontMotor.setPower(0);
                rightFrontMotor.setPower(0);
                leftBackMotor.setPower(0);
                rightBackMotor.setPower(0);

                //Shooter is not set to full power because it shoots too high
                shooter.setPower(0.8);
                //Collector power is set to one to push the ball up
                collector.setPower(1);

                //Rest for 1.5 sec. to allow shooter to reach full speed
                sleep(1500);

                //Lift the stopper out of the way
                stopper.setPosition(0.7);


            }
            else
            {
                //Turns off shooter and puts the stopper back
                stopper.setPosition(0.1);
                shooter.setPower(0);

            }

            //--------------------------------------------------------------------------------------
            //Code to make the hands open and close

            handsMotor.setPower(hands);      //Directly controlled by the controller

            //--------------------------------------------------------------------------------------
            //Elevator code to lift and lower the elevator

            elevatorMotor.setPower(elevator);    //Directly controlled by the controller


            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

}
