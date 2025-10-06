package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Main", group = "Robot")
public class Main extends OpMode {
    private  DcMotor left;
    private DcMotor right;

    @Override
    public void init() {
        left = hardwareMap.get(DcMotorEx.class, "");
        right = hardwareMap.get(DcMotorEx.class, "");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);
        // do stuff here
    }

    @Override
    public void loop() {
        // do even more stuff here (soon™)

        telemetry.addLine("started");
    }
}

class PID {
    private final double kP;
    private final double kI;
    private final double kD;
    private double integral = 0;
    private double previousError = 0;
    private final ElapsedTime loopTime;

    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        loopTime = new ElapsedTime();
        loopTime.reset();
    }

    public double calculatePID(double target, double current) {
        double deltaTime = loopTime.seconds();
        loopTime.reset();

        double error = target - current;
        integral += error * deltaTime;
        double derivative = deltaTime > 0 ? (error - previousError) / deltaTime : 0;
        previousError = error;

        return kP * error + kI * integral + kD * derivative;
    }
}