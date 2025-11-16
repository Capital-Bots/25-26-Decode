/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClasses.testHardware;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;

/**
 * This is our basic TeleOp - just a way for the four wheels on the robot to move in all directions.
 * It is not meant to have any attachments move or even link to odometers - just a basic TeleOp.
 */

@TeleOp(name="testTele", group="teles")

public class testTele extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private testHardware robot = new testHardware();
    boolean isResetRequested = false;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        ThreeDeadWheelLocalizer localizer = new ThreeDeadWheelLocalizer(hardwareMap, 0.00296182028538373,
                new Pose2d(0,0,0));
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot.init(hardwareMap);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            Vector2d input = new Vector2d(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y
            );
            double robotHeading = localizer.getPose().heading.toDouble();
            double xComp =input.x;
            double yComp = -1 * input.y;
            double inputHeading = Math.atan2(yComp, xComp);
            double inputMagnitude = Math.sqrt(xComp*xComp + yComp*yComp);
            inputHeading -= robotHeading;
            isResetRequested = gamepad1.y;
            double finalX = inputMagnitude * Math.cos(inputHeading);
            double finalY = inputMagnitude * Math.sin(inputHeading);
            double turnComponent = gamepad1.right_stick_x;
            boolean rotateForward = gamepad2.a;
            boolean rotateBackward = gamepad2.b;

            boolean slowFront = gamepad2.dpad_up;
            boolean slowBack = gamepad2.dpad_down;
            boolean slowLeft = gamepad2.dpad_left;
            boolean slowRight = gamepad2.dpad_right;

            boolean conveyorOn = true;
            boolean switchConveyorState = gamepad2.x;

            double SPEED_MULTIPLIER = 0.7;


            double normalizingFactor = Math.max(Math.abs(finalY)
                    + Math.abs(finalX) + Math.abs(turnComponent), 1);

            double fl = SPEED_MULTIPLIER * (finalY + finalX + turnComponent) / normalizingFactor;
            double fr = SPEED_MULTIPLIER * (finalY - finalX - turnComponent) / normalizingFactor;
            double bl = SPEED_MULTIPLIER * (finalY - finalX + turnComponent) / normalizingFactor;
            double br = SPEED_MULTIPLIER * (finalY + finalX - turnComponent) / normalizingFactor;

            localizer.update();

            //Slow Movements - DPAD

            if (slowFront){
                fl = 0.35;
                fr = 0.35;
                bl = 0.35;
                br = 0.35;
            }
            else if (slowBack){
                fl = 0.35 * -1;
                fr = 0.35 * -1;
                bl = 0.35 * -1;
                br = 0.35 * -1;
            }
            else if (slowLeft){
                fl = -1 * 0.5;
                fr = 0.5;
                bl = 0.5;
                br = -1 * 0.5;
            }
            else if (slowRight){
                fl = 0.5;
                fr = -1 * 0.5;
                bl = -1*0.5;
                br = 0.5;
            }
            if (isResetRequested) robotHeading = 0;

            if (rotateForward){
                robot.rightRoller.setPower(1);
                robot.leftRoller.setPower(1);
            }else{
                robot.rightRoller.setPower(0);
                robot.leftRoller.setPower(0);
            }
            if (rotateBackward){
                robot.rightRoller.setPower(-1);
                robot.leftRoller.setPower(-1);
            }else{
                robot.rightRoller.setPower(0);
                robot.leftRoller.setPower(0);
            }

            if (conveyorOn){
                robot.rightConveyor.setPower(1);
                robot.leftConveyor.setPower(1);
            }else{
                robot.rightConveyor.setPower(0);
                robot.leftConveyor.setPower(0);
            }
            if (switchConveyorState){
                conveyorOn = !conveyorOn;
            }


            robot.leftFrontDrive.setPower(fl);
            robot.rightFrontDrive.setPower(fr);
            robot.leftBackDrive.setPower(bl);
            robot.rightBackDrive.setPower(br);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Heading: ", robotHeading);
            telemetry.addData("xComp, yComp: ", (""+xComp+", "+yComp));
            telemetry.update();
        }
    }
}