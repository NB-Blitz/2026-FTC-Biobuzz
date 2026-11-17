/* Copyright (c) <2026> <AndyMark>

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of AndyMark nor the names of its contributors may be used to
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


 /*
 * This particular OpMode is a basic auto mode for the Robits StarterBot.
 * This auto mode, launches its preloaded Pollen and drives forward off the wall.
 * This code is based off the sample "RobotAutoDriveByEncoder_Linear"
 * and is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 * that performs the actual movement.
 * This method assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="RobitsStarterBotAuto", group="StarterBot")
//@Disabled

public class RobitsStarterBotAuto extends LinearOpMode {
    
    // The code below is used to set the parameters for the drive method.
    static final double COUNTS_PER_MOTOR_REV = 537.6; // eg: NeveRest 19.2:1
    static final double DRIVE_GEAR_REDUCTION = 1.0;   // No External Gearing
    static final double WHEEL_DIAMETER_INCHES = 3.0;  // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                          (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.5; // # Between 0.1 and 1.0; larger is faster
    static final double TURN_SPEED = 0.5; // # Between 0.1 and 1.0; larger is faster


    // Declare OpMode members.
       private ElapsedTime runtime = new ElapsedTime();
       private Motor frontLeftDrive;
       private Motor frontRightDrive;
       private Motor backLeftDrive;
       private Motor backRightDrive;
       private DcMotor motorCatapult;

       private MecanumDrive drive;
       
       private CRServo rollers;
       
       private Servo stick;

       double leftPower;
       double rightPower;
       
       int delay = 0;
       
       /*
       * These constants control the catapult's wind speed.
       * Adjust as needed.
       */
        private static final double WIND = 0.65;
        
        /*
       * These constants control the intake's roller speeds.
       * Adjust as needed.
       */
        private static final double INTAKE = 1.0;
        private static final double OUTTAKE = -1.0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        /* Initialize the hardware variables. Note that the strings used here as parameters
        * to 'get' must correspond to the names assigned during the robot configuration
        * step (using the FTC Robot Controller app on the phone).
        */
        frontLeftDrive = new Motor(hardwareMap, "front_left_drive"); //TODO: get the IDs for these
        frontRightDrive = new Motor(hardwareMap, "front_right_drive");
        backLeftDrive = new Motor(hardwareMap, "back_left_drive");
        backRightDrive = new Motor(hardwareMap, "back_right_drive");

        motorCatapult = hardwareMap.dcMotor.get("catapult");
        rollers = hardwareMap.crservo.get("rollers");
        stick = hardwareMap.servo.get("stick");

        /* Most robots need the motor on one side to be reversed to drive forward
        * If the robot is driving opposite directions than expected change
        * "REVERSE" to "FORWARD" or vice-versa. This also applies to the catapult.
        */ 
        frontLeftDrive.setInverted(false); //TODO: check if this is correct or flipped
        frontRightDrive.setInverted(false);
        backLeftDrive.setInverted(true);
        backRightDrive.setInverted(true);

        frontLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        /*
        * The code below creates a delay for the start of Auto. This can be useful
        * for coordinating the tipping of the goal with an alliance partner.
        * After initializing the auto mode, use the dpad up/down buttons to increase
        * or decrease the time (in seconds) before the action starts.
        */
        while (!isStarted() && !isStopRequested()){
            telemetry.addData("Press","DPadUp or DPadDown");
            if(gamepad1.dpadUpWasPressed()){
               delay = (delay + 1000);
            } else if (gamepad1.dpadDownWasPressed()){
                if ((delay == 1000)||(delay <1000)){
                    delay = 0;
                } else{
                    delay = (delay - 1000);
                }
            }
           telemetry.addData("Delay (Seconds)", (delay/1000));
           telemetry.update();
        }
     
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
        sleep(delay);
        launch();
        launch();
        launch();
        launch();
        launch();
        encoderDrive(DRIVE_SPEED, 18, 18, 5.0);   //Drive Forward

        // Show the elapsed game time and wheel information
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("FrontLeft Encoder", frontLeftDrive.getCurrentPosition());
        telemetry.addData("FrontRight Encoder", frontRightDrive.getCurrentPosition());
        telemetry.addData("BackLeft Encoder", backLeftDrive.getCurrentPosition());
        telemetry.addData("BackRight Encoder", backRightDrive.getCurrentPosition());
        telemetry.update();
        break;
        }
}

    // Method to perform a relative move, based on encoder counts.
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            newFrontLeftTarget = leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newBackLeftTarget = leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newBackRightTarget = leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            frontLeftDrive.setTargetPosition(newFrontLeftTarget);
            frontRightTarget.setTargetPosition(newFrontRightTarget);
            backLeftDrive.setTargetPosition(newBackLeftTarget);
            backRighttDrive.setTargetPosition(newBackRightTarget);
            rightDrive.setTargetPosition(newRightTarget);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (leftDrive.isBusy() && rightDrive.isBusy())) {

                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            leftDrive.setPower(0);
            rightDrive.setPower(0);

            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250); // Pause after each move
        }
    }
    
    // Method to activate the launching mechanism one time.
    public void launch() {

        if (opModeIsActive()) {
            motorCatapult.setPower(WIND);
            sleep(1000);
            motorCatapult.setPower(0);
            sleep(500);
        }
    }


}
