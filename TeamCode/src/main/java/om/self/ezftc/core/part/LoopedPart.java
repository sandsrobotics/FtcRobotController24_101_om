package om.self.ezftc.core.part;

import om.self.ezftc.core.Robot;
import om.self.task.core.Group;
import om.self.task.core.Task;

public interface LoopedPart extends PartParent{
    class TaskNames {
        public static final String mainLoop = "main loop";
    }

    default void constructLoop(){
        Task t = new Task(TaskNames.mainLoop, getTaskManager());
        t.setRunnable(this::onRun);
        getEventManager().attachToEvent(Robot.Events.START, "start main loop", () -> t.runCommand(Group.Command.START));
    }

    void onRun();
}
