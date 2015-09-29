package org.luminousmonkey.dynamiclayout.config;

public abstract class Motor {

    public abstract String getName();
    public abstract String getDisplayText();

    public abstract void setName(String name);
    public abstract void setDisplayText(String displayText);

    public abstract int getId();
    public abstract int getInitPosition();
    public abstract int getMaxPosition();
    public abstract int getMinPosition();

    public abstract void setId(int id);
    public abstract void setInitPosition(int initPosition);
    public abstract void setMaxPosition(int maxPosition);
    public abstract void setMinPosition(int minPosition);

    public abstract boolean usesAbsolutePosition();
}
