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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 * This program provides a base-level code for the 2026-2027 Robits StarterBot for the BIOBUZZ challenge.
 * By default, this code controls the driving and launching of the robot via one gamepad
 * 
 * The drive style is selectable, but defaults to split arcade
 * Split Arcade: 
 * Driving Forward/Backward Left Joystick Forward/Backward 
 * Turning Left/Right       Right Joystick Left/Right
 *
 * Arcade: 
 * Driving Forward/Backward Left Joystick Forward/Backward 
 * Turning Left/Right       Left Joystick Left/Right
 *
 * Tank:
 * Left Side Drive          Left Joystick Forward/Backward 
 * Right Side Drive         Right Joystick Forward/Backward 
 *
 * The intake controls are by default
 * Intake: Pickup          Left Bumper
 * Intake: Dropoff         Right Bumper
 * 
 * The flower controls are by default
 * Roller: CCW             Left Bumper
 * Roller: CW              Right Bumper
 * Stick: Extend           Right Face Button Left
 * Stick: Retract          Right Face Button Down
 * 
 * The launch controls are by default
 * Release: Catapult       Right Trigger
 * 
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="RobitsStarterBotTeleop", group="StarterBot")
//@Disabled

public class RobitsStarterBotTeleop extends LinearOpMode {

       // Declare OpMode members.
       private ElapsedTime runtime = new ElapsedTime();
       private Motor frontLeftDrive;
       private Motor frontRightDrive;
       private Motor backLeftDrive;
       private Motor backRightDrive;
       private DcMotor motorCatapult;

       private CRServo rollerI;
       private CRServo rollerF;
       
       private Servo stick;

       private MecanumDrive drive;

       IMU imu;

       IMU.Parameters myIMUparameters;

       YawPitchRollAngles robotOrientation;
       
       /*
       * These values control the speed of the intake rollers.
       * Adjust as needed.
       */
       double in = 1.0;
       double out = -1.0;

       /*
       *The following sets up the three styles of drive type
       */

       /*
       * These constants control the catapult's fire speed.
       * Adjust as needed.
       */
        private static final double WIND = 0.65;
        
        /*
        * These costants control the position of the stick flower mechanism.
        */
        private static final double STICKOUT = 0.65;
        private static final double STICKIN = 0.46;

        
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* Initialize the hardware variables. Note that the strings used here as parameters
        * to 'get' must correspond to the names assigned during the robot configuration
        * step (using the FTC Robot Controller app).
        */
        frontLeftDrive = new Motor(hardwareMap, "front_left_drive"); //TODO: get the IDs for these
        frontRightDrive = new Motor(hardwareMap, "front_right_drive");
        backLeftDrive = new Motor(hardwareMap, "back_left_drive");
        backRightDrive = new Motor(hardwareMap, "back_right_drive");
        motorCatapult = hardwareMap.dcMotor.get("catapult");
        rollerI = hardwareMap.crservo.get("rollerI");
        rollerF = hardwareMap.crservo.get("rollerF");
        stick = hardwareMap.servo.get("stick");

        /* Most robots need the motor on one side to be reversed to drive forward
        * If the robot is driving opposite directions than expected change
        * "REVERSE" to "FORWARD" or vice-versa. This also applies to the catapult and rollers.
        */
        frontLeftDrive.setInverted(false); //TODO: check if this is correct or flipped
        frontRightDrive.setInverted(true);
        backLeftDrive.setInverted(false);
        backRightDrive.setInverted(true);

        frontLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        drive = new MecanumDrive(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

        motorCatapult.setDirection(DcMotorSimple.Direction.REVERSE);  
        rollerI.setDirection(DcMotorSimple.Direction.REVERSE);  
        rollerF.setDirection(DcMotorSimple.Direction.FORWARD);  
        
        stick.setPosition(STICKIN); // Defaults the stick flower mechanism to in. 

        imu = hardwareMap.get(IMU.class, "imu");
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(myIMUparameters);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            //Drive Control
            double y = gamepad1.left_stick_y; // Note: pushing stick forward gives negative value
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            
            robotOrientation = imu.getRobotYawPitchRollAngles();

            // Now use these simple methods to extract each angle
            // (Java type double) from the object you just created:
            double heading = robotOrientation.getYaw(AngleUnit.DEGREES);
            telemetry.addData("Heading", heading);

            drive.driveFieldCentric(x, y, rx, heading);
            
            // Flower Stick control
            if (gamepad1.a) {
                stick.setPosition(STICKIN);
            } else if (gamepad1.x) {
                stick.setPosition(STICKOUT);
            }
            
            //Intake control
            if (gamepad1.left_bumper) {
                rollerI.setPower(in);
                rollerF.setPower(in);
            }else if (gamepad1.right_bumper) {
                rollerI.setPower(out);
                rollerF.setPower(out);
            } else {
                rollerI.setPower(0);
                rollerF.setPower(0);
            }
            
            //Launch control
            if (gamepad1.right_trigger > 0) {
                motorCatapult.setPower(WIND);
            } else {
                motorCatapult.setPower(0); // This turns the catapult motor off when not in use
            }            

            // Show the elapsed game time.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            
            // Telemetry data for catapult status
            if (motorCatapult.getPower() == WIND) {
                telemetry.addData("Catapult Status","Wind");
            } else {
                telemetry.addData("Catapult Status","Off");
            }
            
            telemetry.addData("Stick Position", stick.getPosition());
            telemetry.update();
        }
    }
}
