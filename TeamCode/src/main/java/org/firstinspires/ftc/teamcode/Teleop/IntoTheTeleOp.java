/*

Expansion -- DC Motors
port 2 lift_extender
port 3 lift_rotator

Expansion -- Servos
port 0 spin
port 1 claw
port 2 spheal

 */

/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.Teleop;
import androidx.annotation.Nullable;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Vertical:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Rotate:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name="TEST: Omni Linear OpMode", group="Linear OpMode")
//@Disabled
public class IntoTheTeleOp extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor liftRotator = null;
    private DcMotor liftExtender = null;
    private Servo claw = null;
    private CRServo spin = null;
    private CRServo spheal = null;
    //servoController servoController;


//    public boolean normalMode = true ;
//    public boolean sprintMode = false;
//    public boolean crouchMode = false;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        liftRotator = hardwareMap.get(DcMotor.class, "lift_rotator");
        liftExtender = hardwareMap.get(DcMotor.class, "lift_extender");

        claw = hardwareMap.get(Servo.class, "claw");
        //servoController = claw.getController();


        spin = hardwareMap.get(CRServo.class, "spin");
        spheal = hardwareMap.get(CRServo.class, "spheal");

        liftExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftExtender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        liftRotator.setDirection(DcMotor.Direction.FORWARD);
        liftExtender.setDirection(DcMotor.Direction.REVERSE);
//        claw.setDirection(Servo.Direction.FORWARD);
//        spin.setDirection(CRServo.Direction.FORWARD);
//        spheal.setDirection(Servo.Direction.FORWARD);

//        liftRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double vertical = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            ElapsedTime armTime = new ElapsedTime();

//            claw.setPosition(1);

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = (vertical + lateral + rotate);
            double rightFrontPower = (vertical - lateral - rotate);
            double leftBackPower = (vertical - lateral + rotate);
            double rightBackPower = (vertical + lateral - rotate);

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            // finds wheel with the highest power value
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            // divides power value of all wheels by highest power value
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            if (gamepad1.left_trigger > 0) { // runs if left trigger is pressed (crawl mode)
                leftFrontPower /= 4;
                rightFrontPower /= 4;
                leftBackPower /= 4;
                rightBackPower /= 4;
            } else if (gamepad1.right_trigger == 0) { // runs if neither trigger is pressed (normal mode)
                leftFrontPower /= 2;
                rightFrontPower /= 2;
                leftBackPower /= 2;
                rightBackPower /= 2;
            } // else situation (right trigger pressed / sprint mode) ...
            // ... does not require code, take default values which are fastest


//           while (gamepad1.left_trigger>0) {
//                leftFrontPower  = vertical + lateral + rotate;
//            }
//            if (gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0) {
//                float armPower = (-gamepad2.left_trigger + gamepad2.right_trigger);
//                if (armPower > 0.8) {
//                    armPower = 0.8F;
//                }
//                liftRotator.setPower(armPower);
//            } else {
//                liftRotator.setPower(0);
//            }

            // triggers on gamepad 2 -- setting power for lift rotator (DC motor)
            if (gamepad2.left_trigger > 0) {
                liftRotator.setPower(Range.clip(-gamepad2.left_trigger, -0.5, 0));
            } else if (gamepad2.right_trigger > 0) {
                liftRotator.setPower(Range.clip(gamepad2.right_trigger, 0, 1));
            }
//            else {
//                liftRotator.setPower(0);
//            }


            // a/b on gamepad 2 -- setting power for claw (CR servo)
            if (gamepad2.a) { // to open
                claw.setPosition(0.95);
            } else if (gamepad2.b) { // to close
                claw.setPosition(0.7);
            }
//            else {
//                claw.setPosition(0.875); // to test if power is off to claw
//            }

            if (gamepad2.x) {
                spin.setPower(-0.5);
            } else if (gamepad2.y) {
                spin.setPower(0.5);
            } else {
                spin.setPower(0);
            }

//             bumpers on gamepad 2 -- setting position of spheal (position servo)
            if (gamepad2.left_bumper) { // untuck
                spheal.setPower(0.4); // used to be 0.5
            } else if (gamepad2.right_bumper) { // tuck
                spheal.setPower(-0.5); //try 0.9
            } else {
                spheal.setPower(0);
            }

            // left joystick on gamepad 2 -- setting power of lift extender (DC motor)
            if (gamepad2.left_stick_y > 0) {
                liftExtender.setPower(-0.8);
//                // reverse or back motion of arm
//                //liftExtender.setPower(-1);
//                liftExtender.setPower(-0.8);
////                if (liftExtender.getCurrentPosition() < 100) {
////                    liftExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////                }
//            } else if (liftExtender.getCurrentPosition() > 2500) {
//                liftExtender.setPower(0.005);
//            } else if (gamepad2.left_stick_y < 0) {
//                // forward or extension motion of arm
//                //liftExtender.setPower(1);
//                if (liftExtender.getCurrentPosition() > 2500) {
//                    liftExtender.setPower(0.05);
//                }
//                else {
//                    liftExtender.setPower(0.8);
//                }
            } else if (gamepad2.left_stick_y < 0) {
                liftExtender.setPower(0.8);
            } else {
                liftExtender.setPower(0);
            }

//                // maybe change below code into 'if elseif else if doesnt work
//            if (normalMode) {
//                leftFrontPower = (vertical + lateral + rotate) / 2;
//                rightFrontPower = (vertical - lateral - rotate) / 2;
//                leftBackPower = (vertical - lateral + rotate) / 2;
//                rightBackPower = (vertical + lateral - rotate) / 2;
//            }
//            if (sprintMode>0) {
//                leftFrontPower = (vertical + lateral + rotate);
//                rightFrontPower = (vertical - lateral - rotate);
//                leftBackPower = (vertical - lateral + rotate);
//                rightBackPower = (vertical + lateral - rotate);
//            }
//            if (crouchMode>0) {
//                leftFrontPower = (vertical + lateral + rotate) / 4;
//                rightFrontPower = (vertical - lateral - rotate) / 4;
//                leftBackPower = (vertical - lateral + rotate) / 4;
//                rightBackPower = (vertical + lateral - rotate) / 4;
//            }
            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.
            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */


            // sending calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("liftExtender position", liftExtender.getCurrentPosition());
            telemetry.addData("liftRotator position", liftRotator.getCurrentPosition());
            telemetry.update();
        }
    }}

