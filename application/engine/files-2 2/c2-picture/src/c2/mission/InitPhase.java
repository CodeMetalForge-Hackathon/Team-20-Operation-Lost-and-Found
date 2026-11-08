package c2.mission;

import c2.input.Command;

import java.time.Duration;
import java.time.Instant;

/** Plan the next route from the current belief, issue the mission order, launch. */
public class InitPhase implements MissionPhase {
    @Override public String name() { return "INIT: planning sortie"; }

    @Override
    public void enter(MissionController controller) { controller.launchNewSortie(); }

    @Override
    public MissionPhase onTick(MissionController controller, Instant now, Duration elapsed) {
        return new ExecutePhase();
    }

    @Override
    public MissionPhase onCommand(MissionController controller, Command command) { return this; }
}
