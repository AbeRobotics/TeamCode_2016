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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareRobot;



/**
 * Created by aedanwen on 2017-02-01.
 */
@TeleOp(name = " Robot Main Teleop", group = "Robot")
//@Disabled
public class RobotTeleopMain extends OpMode{

    HardwareRobot robot = new HardwareRobot();
    boolean launcher_running;
    boolean brushes_running;



    public void init(){

        robot.init(hardwareMap);

    }

    public void start(){


    }

    public void init_loop(){

    }

    public void loop() {

        double launcherPower = gamepad1.right_trigger;
        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;
        boolean brushPowerback = gamepad1.right_bumper;
        boolean brushPowerbackfront = gamepad1.left_bumper;


        robot.wheels_left.setPower(left);
        robot.wheels_right.setPower(right);
        robot.launcher.setPower(launcherPower);

        if(brushPowerback){
            robot.brushes.setPower(1);
        }
        robot.wheels_left.setPower(left);
        robot.wheels_right.setPower(right);

        if(brushPowerbackfront) {
            robot.brushes.setPower(-1);
        }







        if(gamepad1.y){

            if(brushes_running){

                brushes_running = false;
                robot.brushes.setPower(0);
            }else{
                brushes_running = true;
            }
        }

        if(gamepad1.a){

            //Run Launcher for 2 seconds

        }




        telemetry.addData("Encoder Left", robot.wheels_left.getCurrentPosition());
        telemetry.addData("Encoder Right", robot.wheels_right.getCurrentPosition());
        telemetry.addData("Laucnher", robot.launcher.getPower());
        telemetry.addData("Brushes Running", brushes_running);



    }


    public void stop() {

    }


}


































