package org.firstinspires.ftc.teamcode.commands;

import java.util.ArrayList;
import java.util.Iterator;

/**
 * Created by David Austin on 11/22/2016.
 */

public class CommandGroup extends BasicCommand {
    ArrayList<BasicCommand> commands = new ArrayList<BasicCommand>();
    Iterator<BasicCommand> iterator;
    public void addCommand(BasicCommand cmd) {
        commands.add(cmd);
    }

    public void init() {
        iterator = commands.iterator();
        while (iterator.hasNext()) {
            iterator.next().init();
        }
    }

    public void execute() {
        iterator = commands.iterator();
        ArrayList<BasicCommand> toRemove = new ArrayList<BasicCommand>();
        while (iterator.hasNext()) {
            BasicCommand cmd = iterator.next();
            if (cmd.isFinished()) {
                cmd.stop();
                toRemove.add(cmd);
            } else {
                cmd.execute();
            }
        }
        Iterator removeIterator = toRemove.iterator();
        while (removeIterator.hasNext()) {
            commands.remove(removeIterator.next());
        }
    }

    public boolean isFinished() {
        return commands.isEmpty();
    }
}
