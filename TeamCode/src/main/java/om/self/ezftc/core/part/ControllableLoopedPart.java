package om.self.ezftc.core.part;

import java.util.function.Supplier;

import om.self.task.core.Group;

public abstract class ControllableLoopedPart<PARENT extends PartParent, SETTINGS, HARDWARE, CONTROL> extends ControllablePart<PARENT, SETTINGS, HARDWARE, CONTROL> implements LoopedPart{
    public ControllableLoopedPart(PARENT parent, String name, Supplier<CONTROL> baseController) {
        super(parent, name, baseController);
        constructLoop();
        constructControllable();
    }

    public ControllableLoopedPart(PARENT parent, String name, Group taskManager, Supplier<CONTROL> baseController) {
        super(parent, name, taskManager, baseController);
        constructLoop();
        constructControllable();
    }

    public ControllableLoopedPart(PARENT parent, String name) {
        super(parent, name);
        constructLoop();
        constructControllable();
    }

    public ControllableLoopedPart(PARENT parent, String name, Group taskManager) {
        super(parent, name, taskManager);
        constructLoop();
        constructControllable();
    }

}
