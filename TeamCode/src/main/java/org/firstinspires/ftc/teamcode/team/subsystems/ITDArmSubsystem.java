package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.annotations.PIDSVA;
import org.firstinspires.ftc.teamcode.lib.control.ControlConstants;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.motion.IMotionProfile;
import org.firstinspires.ftc.teamcode.lib.motion.ResidualVibrationReductionMotionProfilerGenerator;
import org.firstinspires.ftc.teamcode.team.DbgLog;
import org.firstinspires.ftc.teamcode.team.states.ITDArmStateMachine;

@PIDSVA(name = "Extend",
        P = 0.4d,
        I = 0d,
        D = 0d,
        S = 0.10d,
        V = (1 - 0.10d) / 15d,
        A = 0d
)

@PIDSVA(name = "Retract",
        P = 0.08d,
        I = 0d,
        D = 0d,
        S = 0.06d,
        V = 1 / 55d,
        A = 0d
)

public class ITDArmSubsystem implements ISubsystem<ITDArmStateMachine, ITDArmStateMachine.State>{

    private static final ControlConstants EXTEND_CONTROL_CONSTANTS;
    private static final ControlConstants RETRACT_CONTROL_CONSTANTS;
    private static ITDArmStateMachine itdArmStateMachine;
    private RevMotor leftArm;
    private RevMotor rightArm;
    private static IMotionProfile extensionProfile = null;
    private static double setpoint = 0d;
    private static double desiredSetpoint = 0d;
    private double lastError;
    private double runningSum;

    static {
        new Thread(ResidualVibrationReductionMotionProfilerGenerator::init).start();
        PIDSVA[] controllers = Feeder.class.getAnnotationsByType(PIDSVA.class);
        if(controllers.length == 2) {
            PIDSVA extendController;
            PIDSVA retractController;
            if(controllers[0].name().equals(ITDArmStateMachine.State.EXTEND.getName())) {
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

        ITDArmStateMachine.setRunExtension((setpoint) -> {
            if(getExtensionProfile() != null) {
                getExtensionProfile().start();
                ITDArmSubsystem.setpoint = setpoint;
            }
        });
    }


    public ITDArmSubsystem(RevMotor leftArm, RevMotor rightArm) {
        setITDArmStateMachine(new ITDArmStateMachine(this));
        setLeftArm(leftArm);
        setRightArm(rightArm);
        setLastError(0d);
        resetRunningSum();
    }

    public void resetRunningSum() {
        setRunningSum(0d);
    }

    @Override
    public String getName() {
        return "ITDArmSubsystem";
    }

    public void update(double dt) {
        getITDArmStateMachine().update(dt);

        double error                = getSetpoint() - getLeftArm().getPosition(); //error in how high the lift goes
        double setpointVelocity     = 0d;
        double setpointAcceleration = 0d;
        if(getExtensionProfile() != null && !getExtensionProfile().isDone()) {
            setpointVelocity     = getExtensionProfile().getVelocity();
            setpointAcceleration = getExtensionProfile().getAcceleration();
        }

        setRunningSum(getRunningSum() + error * dt);
        double output;
        if(getITDArmStateMachine().getState().equals(ITDArmStateMachine.State.EXTEND)) {
            output = getExtendControlConstants().getOutput(dt, error, getLastError(), getRunningSum(), setpointVelocity, setpointAcceleration, false);
            output += getExtendControlConstants().kS();

        } else {
            output = getRetractControlConstants().getOutput(dt, error, getLastError(), getRunningSum(), setpointVelocity, setpointAcceleration, true);

                }
        setLastError(error);

        final double kP = 0.001d;
        double relativeError = getLeftArm().getPosition() - getRightArm().getPosition();
        double relativeOutput = kP * relativeError;
        getLeftArm().setPower(output);
        getRightArm().setPower(output + relativeOutput);
    }

    public void retract() {
        resetRunningSum();
        setSetpoint(0d);
        setDesiredSetpoint(0d);
    }

    public boolean closeToSetpoint(double threshold) {
        return Math.abs(getSetpoint() - getLeftArm().getPosition()) <= threshold;
    }

    public static ITDArmStateMachine getITDArmStateMachine() {
        return itdArmStateMachine;
    }

    public static void setITDArmStateMachine(ITDArmStateMachine itdArmStateMachine) {
        ITDArmSubsystem.itdArmStateMachine = itdArmStateMachine;
    }

    public RevMotor getLeftArm() {return leftArm;}
    public void setLeftArm(RevMotor leftArm) {
        this.leftArm = leftArm;
    }

    public RevMotor getRightArm() {
        return rightArm;
    }

    public void setRightArm(RevMotor rightArm) {this.rightArm = rightArm;}

    public static double getSetpoint() {
        return setpoint;
    }

    public void setSetpoint(double setpoint) {
        setDesiredSetpoint(setpoint);
        if(setpoint != getSetpoint() && (getExtensionProfile() == null || getExtensionProfile().isDone())) {
            if(setpoint != 0d) {
                //Extending
                getITDArmStateMachine().updateState(ITDArmStateMachine.State.EXTEND);
                this.setpoint = setpoint;
            } else {
                //Retracting
                getITDArmStateMachine().updateState(ITDArmStateMachine.State.RETRACT);
                setExtensionProfile(new ResidualVibrationReductionMotionProfilerGenerator(
                        getLeftArm().getPosition(), -getLeftArm().getPosition(), 25d, 50d
                ));
                this.setpoint = setpoint;
            }
        }
        if(getExtensionProfile() == null){
            DbgLog.msg("EXTENSION PROFILE NULL");
        } //Debugging purposes 12/20
        else {
            DbgLog.msg("EXTENSION PROFILE");
        }
    }

    public static IMotionProfile getExtensionProfile() {
        return extensionProfile;
    }

    public static ControlConstants getExtendControlConstants() {
        return EXTEND_CONTROL_CONSTANTS;
    }

    public static void setExtensionProfile(IMotionProfile extensionProfile) {
        ITDArmSubsystem.extensionProfile = extensionProfile;
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


    @Override
    public ITDArmStateMachine getStateMachine() {
        return itdArmStateMachine;
    }

    @Override
    public ITDArmStateMachine.State getState() {
        return null;
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }
}
