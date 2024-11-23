package org.firstinspires.ftc.teamcode.team.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;
import org.firstinspires.ftc.teamcode.lib.util.Time;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

public class ITDClawArmStateMachine extends TimedState<ITDClawArmStateMachine.State> {
    public ITDClawArmStateMachine() {
        super(State.IDLE);
    }

    @Override
    public String getName() {
        return "Claw Arm State Machine";
    }

    @Override
    protected Time getStateTransitionDuration() {
        return new Time(10d, TimeUnits.MILLISECONDS);
    }

    public enum State implements Namable {
        PICKUP("Open",-0.3d), DROP("Close", 0.4d), IDLE("Close", 0.75d);

        private final String name;
        private final double Position;

        State(final String name, final double pPosition) {
            this.name          = name;
            this.Position  = pPosition;
        }

        public double getPosition() {
            return Position;
        }

        @Override

        public String getName() {
            return name;
        }
    }
}