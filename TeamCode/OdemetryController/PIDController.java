// PIDController.java
// Ported from your PID.py. See original: path_planning.py / PID.py. :contentReference[oaicite:2]{index=2} :contentReference[oaicite:3]{index=3}

package org.firstinspires.ftc.teamcode.Autonomous.OdemetryController;

public class PIDController {
    private double Kp, Ki, Kd;
    private double dt;
    private double setpoint;

    private double error = 0.0;
    private double previousError = 0.0;
    private double integralError = 0.0;
    private double derivativeError = 0.0;
    private double output = 0.0;

    public PIDController(double Kp, double Ki, double Kd, double dt, double setpoint) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.dt = dt;
        this.setpoint = setpoint;
    }

    /**
     * Call this each control loop to get the controller output for current measurement 'pos'.
     * It returns Kp*error + Ki*integral + Kd*derivative
     */
    public double step(double pos) {
        error = setpoint - pos;
        integralError += error * dt;
        derivativeError = (error - previousError) / dt;
        previousError = error;
        output = Kp * error + Ki * integralError + Kd * derivativeError;
        return output;
    }

    public void setSetpoint(double sp) { this.setpoint = sp; }
    public double getSetpoint() { return setpoint; }

    public void reset() {
        error = previousError = integralError = derivativeError = output = 0.0;
    }

    public double[] getTunings() {
        return new double[] {Kp, Ki, Kd};
    }

    public void setTunings(double Kp, double Ki, double Kd) {
        this.Kp = Kp; this.Ki = Ki; this.Kd = Kd;
    }
}
