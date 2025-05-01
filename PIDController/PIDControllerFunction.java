package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class PIDControllerFunction extends LinearOpMode {
    
    double kP = 33.45; 
    double kI = 33.5;
    double kD = 3.35;
    
    ElapsedTime timer = new ElapsedTime();
    double lastError = 0;
    double integralSum = 0;
    
    @Override
    public void runOpMode() {
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        while (opModeIsActive()) { 

        // todo: write your code here
    
        }
    }
    
    /* Function called PIDControl takes two variables in and outputs 
    the sum of the Proportional, Derivative, and Integral (PID)
    
    INPUTS: Reference: Target goal for motor to achieve 
            State: Encoder/sensor input to track motor position 
            
    OUTPUT: Sum of the Proportional, Integral, and Derivative term
    */
    
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
