package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.*;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by aedanwen on 2017-02-01.
 */

public class HardwareRobot {

    public DcMotor brushes;
    public DcMotor wheels_left;
    public DcMotor wheels_right;
    public DcMotor launcher;
    HardwareMap hwMap  =  null;


        /* Initialize standard Hardware interfaces */
        public void init(HardwareMap ahwMap) {
            hwMap = ahwMap;

            brushes = hwMap.dcMotor.get("Brushes");
            launcher = hwMap.dcMotor.get("Launcher");
            wheels_left = hwMap.dcMotor.get("Left Wheels");
            wheels_right = hwMap.dcMotor.get("Right Wheels");




            brushes.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wheels_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheels_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            wheels_left.setDirection(DcMotorSimple.Direction.FORWARD);
            wheels_right.setDirection(DcMotorSimple.Direction.REVERSE);
            brushes.setDirection(DcMotorSimple.Direction.REVERSE);
            launcher.setDirection(DcMotorSimple.Direction.REVERSE);


    }




}
