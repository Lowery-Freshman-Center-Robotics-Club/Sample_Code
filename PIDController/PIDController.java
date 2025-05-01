/*
Copyright 2025 FIRST Tech Challenge Team MINI

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp

public class PIDController extends LinearOpMode {
    
    /* 
    Best values found round 1 include the following:
    kP = 33.25
    kI = 33.5
    kD = 3.3
    angleSetPoint = -89
    */ 
    
    /* 
    Best values found round 1 include the following:
    kP = 33.45
    kI = 33.5
    kD = 3.35
    angleSetPoint = -0.5
    */ 
    
    double kP = 33.45; 
    double kI = 33.5;
    double kD = 3.35;
    double angleSetPoint = -0.5; 
    double angleOffset = 0;
    
    ElapsedTime timer = new ElapsedTime();
    double lastError = 0;
    double integralSum = 0;


    @Override
    public void runOpMode() {
        
        // Retreieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match the robot 
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        DcMotorEx M1 = (DcMotorEx) hardwareMap.dcMotor.get("M1");
        DcMotorEx M2 = (DcMotorEx) hardwareMap.dcMotor.get("M2");
        
        M1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M1.setDirection(DcMotor.Direction.REVERSE);
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double botAngle = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
            double power = PIDControl((angleSetPoint + angleOffset), botAngle); 
            double y = gamepad1.left_stick_y;
            double rotX = gamepad1.right_stick_x;
            
            M1.setVelocity(power + (rotX*250)); 
            M2.setVelocity(power - (rotX*250));
            
            if (y >= .25) {
                angleOffset = 2.5;
            } else if (y <= -.25) {
                angleOffset = -2.5;
            } else {
                angleOffset = 0;
            }
            
            telemetry.addData("botAngle:", botAngle);
            telemetry.addData("motorPower:", power);
            telemetry.update();
        }
    }
    
    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum = error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        
        timer.reset();
        
        double output = (error * kP) + (derivative * kD) + (integralSum * kI);
        return output;
    }
}
