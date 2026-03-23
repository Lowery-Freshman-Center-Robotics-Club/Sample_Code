/*  Information and Instructions 
    
    This code allows the user to toggle the buttons A, B and Y to modify in realtime
    the P, I, and D varables for a PID controller. This allows the user to test different 
    values and see how the system reacts allow the user to fine tune the control of the 
    system. 
    
    To use this code follow the instructions below:
    1. Setup the variables to match your desired motor
        1A. Enter motor ticks per rev and gear ratio
        1B. Enter motor name pulled from robot conifguration 
    2. Verify the position 1 and 2 values are valid for your application
    3. Build code and run on robot
    4. After running the code your robot should try to move the motor to position 1
    5. To modify the P value press A
        5A. You should see A Toggle State go from "False" to "True"
    6. In this case you can use the right and left bumpers to add or subtract the change value 
        6A. Default value is 0.0001
    7. To modify the I value press B
    8. To modify the D value press Y
    9. To turn off modification of P, I or D values press the A, B or Y button to set the toggle state to false
        9A. The variable will only change if the toggle state is set to "True"
    10. You can use the DPAD up and down buttons to toggle between two positions 
    11. Once done with tuning, make note of the P, I and D values on the screen. These values will not save once code is stoped!
*/


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="PID_Tuner", group="PID_Tuner")

public class PID_Tuner extends LinearOpMode {
    
    // Setup variables used for timer 
    ElapsedTime timer = new ElapsedTime();
    double lastError = 0;
    double integralSum = 0;
    private ElapsedTime runtime = new ElapsedTime();
    
    // Setup variables used for motor positioning 
    // targetMotorPosition1 is the first position you want the motor to move to in Degrees 
    int targetMotorPosition1 = 0; 
    int targetMotorPosition2 = 90; 
    
    // Variable used to set the name of the target motor, pull the name of the motor from the configuration in the driver station app
    // Enter the name within the quotes for example "Enter Name Here"
    String motorName = "elbow";
    
    // Setup variables used for motor gearing 
    // Enter the gear ratio of the motor gearbox and the ticks per revolution of the motor
    // Quick reference: Rev and GoBuilda motors are 28 ticks per rev
    int motorGearRatio = 12;
    int motorTicksPerRev = 28;
    int outputTicksPerRev = motorGearRatio * motorTicksPerRev;
    
    // Math to convert the motor positions in degrees to ticks 
    int targetPosition1Ticks = (outputTicksPerRev*targetMotorPosition1)/360;
    int targetPosition2Ticks = (outputTicksPerRev*targetMotorPosition2)/360;
    int targetPosition = 0;
    
    // Setup variables for toggles 
    boolean aToggle = false;
    boolean lastAState = false;
    
    boolean bToggle = false;
    boolean lastBState = false;
    
    boolean yToggle = false;
    boolean lastYState = false;
    
    // Setup variable to set the increments to add/subtract the P I and D values by
    double changeValue = .0001;
    
    // Setup variables for our P I and D values 
    double kP = 0; 
    double kI = 0;
    double kD = 0;

    @Override
    public void runOpMode() {
        
        // Declare our motor and setup to run without encoder 
        DcMotorEx motor = (DcMotorEx) hardwareMap.dcMotor.get(motorName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            // Calculate our motor power by using the PID function and use setVelocity command to power motor 
            double power = PIDControl(targetPosition, motor.getCurrentPosition());
            motor.setVelocity(power);
            
            // Setup logic to track our toggles for A, B and Y buttons 
            if (gamepad1.a && !lastAState) {
                aToggle = !aToggle;
            }
            lastAState = gamepad1.a;
            
            if (gamepad1.b && !lastBState) {
                bToggle = !bToggle;
            }
            lastBState = gamepad1.b;
            
            if (gamepad1.y && !lastYState) {
                yToggle = !yToggle;
            }
            lastYState = gamepad1.y;
            
            // Look for user to hit DPAD UP/DOWN to trigger different motor positions 
            if (gamepad1.dpad_up) {
                targetPosition = targetPosition1Ticks;
            }
             if (gamepad1.dpad_down) {
                targetPosition = targetPosition2Ticks;
            }           
            
            // Setup P I and D values to add or subract the change value 
            // Look at a b and Y toggle states to know when to add or subtract for each channel 
            if (aToggle) {
                if (gamepad1.right_bumper) {
                    kP = kP + changeValue;
                } else if (gamepad1.left_bumper) {
                    kP = kP - changeValue;
                }
            }
            
            if (bToggle) {
                if (gamepad1.right_bumper) {
                    kI = kI + changeValue;
                } else if (gamepad1.left_bumper) {
                    kI = kI - changeValue;
                }
            }
            
            if (yToggle) {
                if (gamepad1.right_bumper) {
                    kD = kD + changeValue;
                } else if (gamepad1.left_bumper) {
                    kD = kD - changeValue;
                }
            }
        
            // Setup telemetry to send messages to driver station 
            telemetry.addData("motorPosition:", motor.getCurrentPosition());
            telemetry.addData("motorPower:", power);
            telemetry.addData("A Toggle State:", aToggle);
            telemetry.addData("B Toggle State:", bToggle);
            telemetry.addData("Y Toggle State:", yToggle);
            telemetry.addData("P:", kP);
            telemetry.addData("I:", kI);
            telemetry.addData("D:", kD);
            telemetry.update();
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
