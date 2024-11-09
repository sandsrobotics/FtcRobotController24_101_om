package om.self.ezftc.core.part;

import om.self.task.core.Group;

public abstract class LoopedPartImpl<PARENT extends PartParent, SETTINGS, HARDWARE> extends Part<PARENT, SETTINGS, HARDWARE> implements LoopedPart{

    /**
     * A constructor for Part that creates task and event managers attached to the parents task and event managers. This also create a loop that will run forever
     * @param parent the parent of this part(CAN NOT BE NULL)
     * @param name   the name of this part (used to register an event manager and task manager so it must be unique in the scope of the parent)
     */
    public LoopedPartImpl(PARENT parent, String name) {
        super(parent, name);
        constructLoop();
    }

    /**
     * A constructor for Parts that create an event managers attached to the parents and a task managers attached to the custom task manager. This also create a loop that will run forever
     * @param parent      the parent of this part(CAN NOT BE NULL)
     * @param name        the name of this part (used to register an event manager and task manager so it must be unique in the scope of the parent)
     * @param taskManager the custom task manager that is the parent for this parts task manager
     */
    public LoopedPartImpl(PARENT parent, String name, Group taskManager) {
        super(parent, name, taskManager);
        constructLoop();
    }
}
