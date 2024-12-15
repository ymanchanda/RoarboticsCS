package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.annotations.PIDSVA;
import org.firstinspires.ftc.teamcode.lib.control.ControlConstants;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.motion.IMotionProfile;
import org.firstinspires.ftc.teamcode.team.DbgLog;
import org.firstinspires.ftc.teamcode.team.states.ITDArmStateMachine;

@PIDSVA(name = "Extend",
        P = 0.4d,
        I = 0d,
        D = 0.00d,
        S = 0.05d,
        V = (1 - 0.05) / 15d,
        A = 0d
)

@PIDSVA(name = "Retract",
        P = 0.08d,
        I = 0d,
        D = 0d,
        S = 0.05d,
        V = 1 / 55d,
        A = 0d
)
public class ITDArmSubsystem implements ISubsystem<ITDArmStateMachine, ITDArmStateMachine.State> {
    private static final ControlConstants EXTEND_CONTROL_CONSTANTS;
    private static final ControlConstants RETRACT_CONTROL_CONSTANTS;

    private static ITDArmStateMachine itdArmStateMachine;
    private RevMotor arm;

    private static IMotionProfile extensionProfile = null;
    private static double setpoint = 0d;
    private static double desiredSetpoint = 0d;
    private double lastError;
    private double runningSum;

    static {
        //new Thread(ResidualVibrationReductionMotionProfilerGenerator::init).start();
        PIDSVA[] controllers = ITDArmSubsystem.class.getAnnotationsByType(PIDSVA.class);
        if(controllers.length == 2) {
            PIDSVA extendController;
            PIDSVA retractController;
            if(controllers[0].name().equals(ITDArmStateMachine.State.UP.getName())) {
                extendController  = controllers[0];
                retractController = controllers[1];
            } else {
                extendController  = controllers[1];
                retractController = controllers[0];
            }

            EXTEND_CONTROL_CONSTANTS = new ControlConstants(
                    extendController.P(), extendController.I(), extendController.D(),
                    extendController.S(), extendController.V(), extendController.A()
            );

            RETRACT_CONTROL_CONSTANTS = new ControlConstants(
                    retractController.P(), retractController.I(), retractController.D(),
                    retractController.S(), retractController.V(), retractController.A()
            );
        } else {
            EXTEND_CONTROL_CONSTANTS  = new ControlConstants();
            RETRACT_CONTROL_CONSTANTS = new ControlConstants();
        }
    }

    public ITDArmSubsystem(RevMotor arm) {
        setArmStateMachine(new ITDArmStateMachine(this));
        setArm(arm);
        setLastError(0d);
        resetRunningSum();
    }

    public void resetRunningSum() {
        setRunningSum(0d);
    }

    @Override
    public ITDArmStateMachine getStateMachine() {
        return itdArmStateMachine;
    }

    @Override
    public ITDArmStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {
    }

    @Override
    public void stop() {
        getArm().setPower(0d);
        getArm().resetEncoder();
        setSetpoint(0d);
        setDesiredSetpoint(0d);
    }

    @Override
    public String getName() {
        return "Arm";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        //getStateMachine().update(dt);
        getArmStateMachine().update(dt);

        double error                = getSetpoint() - getArm().getPosition();
        double setpointVelocity     = 0d;
        double setpointAcceleration = 0d;
        if(getExtensionProfile() != null && !getExtensionProfile().isDone()) {
            setpointVelocity     = getExtensionProfile().getVelocity();
            setpointAcceleration = getExtensionProfile().getAcceleration();
        }

        DbgLog.msg("Error: " + String.valueOf(error));

        setRunningSum(getRunningSum() + error * dt);
        double output;
        if(getArmStateMachine().getState().equals(ITDArmStateMachine.State.UP)) { //changed from getstate to getdesiredstate
            output = getExtendControlConstants().getOutput(dt, error, getLastError(), getRunningSum(), setpointVelocity, setpointAcceleration, false);
            output += getExtendControlConstants().kS();
        }
        else {
            output = getRetractControlConstants().getOutput(dt, error, getLastError(), getRunningSum(), setpointVelocity, setpointAcceleration, true);
        }

        DbgLog.msg("Output: " + String.valueOf(output));

        setLastError(error);
        getArm().setPower(output);

    }

    public void extend(Double set) {
        resetRunningSum();
        setSetpoint(set);
    }

    public void retract() {
        resetRunningSum();
        setSetpoint(0d);
        setDesiredSetpoint(0d);
    }

    public boolean closeToSetpoint(double threshold) {
        return Math.abs(getSetpoint() - getArm().getPosition()) <= threshold;
    }

    public static ITDArmStateMachine getArmStateMachine() {
        return itdArmStateMachine;
    }

    public static void setArmStateMachine(ITDArmStateMachine armSM) {
        ITDArmSubsystem.itdArmStateMachine = armSM;
    }

    public RevMotor getArm() {
        return arm;
    }

    public void setArm(RevMotor arm) {
        this.arm = arm;
    }

    public static double getSetpoint() {
        return setpoint;
    }

    public void setSetpoint(double setpoint) {
        setDesiredSetpoint(setpoint);
        if(setpoint != getSetpoint() && (getExtensionProfile() == null || getExtensionProfile().isDone())) {
            if(setpoint != 0d) {
                //Extending
                getArmStateMachine().updateState(ITDArmStateMachine.State.UP);
                this.setpoint = setpoint;
            } else {
                //Retracting
                getArmStateMachine().updateState(ITDArmStateMachine.State.DOWN);
                this.setpoint = setpoint;
                //setExtensionProfile(new ResidualVibrationReductionMotionProfilerGenerator(
                //        getLift().getPosition(), -getLift().getPosition(), 25d, 50d
                //));
            }
        }
    }

    public static IMotionProfile getExtensionProfile() {
        return extensionProfile;
    }

    public static void setExtensionProfile(IMotionProfile extensionProfile) {
        ITDArmSubsystem.extensionProfile = extensionProfile;
    }

    public static ControlConstants getExtendControlConstants() {
        return EXTEND_CONTROL_CONSTANTS;
    }

    public static ControlConstants getRetractControlConstants() {
        return RETRACT_CONTROL_CONSTANTS;
    }

    public double getLastError() {
        return lastError;
    }

    public void setLastError(double lastError) {
        this.lastError = lastError;
    }

    public double getRunningSum() {
        return runningSum;
    }

    public void setRunningSum(double runningSum) {
        this.runningSum = runningSum;
    }

    public static double getDesiredSetpoint() {
        return desiredSetpoint;
    }

    public static void setDesiredSetpoint(double desiredSetpoint) {
        ITDArmSubsystem.desiredSetpoint = desiredSetpoint;
    }
}
