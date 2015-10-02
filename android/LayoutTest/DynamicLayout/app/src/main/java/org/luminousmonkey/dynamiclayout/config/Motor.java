package org.luminousmonkey.dynamiclayout.config;

public class Motor {
    private String name;
    private String displayText;
    private int id;
    private int initPosition;
    private int minPosition, maxPosition;
    private boolean mUsesAbsolutePosition;

    public Motor() {
        mUsesAbsolutePosition = false;
    }

    public void setUsesAbsolutePosition(boolean useAbsolutePosition) {
        mUsesAbsolutePosition = useAbsolutePosition;
    }

    public void setDisplayText(String displayName) {
        this.displayText = displayName;
    }

    public String getDisplayText() {
        return this.displayText;
    }

    public void setName(String name) {
        this.name = name;
    }

    public String getName() {
        return this.name;
    }

    public void setId(int id) {
        this.id = id;
    }

    public int getId() {
        return this.id;
    }

    public void setInitPosition(int initPosition) {
        this.initPosition = initPosition;
    }

    public int getInitPosition() {
        return this.initPosition;
    }

    public void setMinPosition(int minPosition) {
        this.minPosition = minPosition;
    }

    public int getMinPosition() {
        return this.minPosition;
    }

    public void setMaxPosition(int maxPosition) {
        this.maxPosition = maxPosition;
    }

    public int getMaxPosition() {
        return this.maxPosition;
    }

    public boolean usesAbsolutePosition() {
        return mUsesAbsolutePosition;
    }

    @Override
    public String toString() {
        return "Name: " + this.getName() + ", " +
            "ID: " + this.getId() + ", " +
            "InitPos: " + this.getInitPosition() + ", " +
            "MinPos: " + this.getMinPosition() + ", " +
            "MaxPos: " + this.getMaxPosition() + "," +
            "Absolute Pos: " + this.usesAbsolutePosition() + ".";
    }
}
