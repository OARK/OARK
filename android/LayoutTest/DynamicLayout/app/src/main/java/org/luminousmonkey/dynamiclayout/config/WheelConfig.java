package org.luminousmonkey.dynamiclayout.config;

public class WheelConfig {
    private String name;
    private int id;
    private int initPosition;
    private int minPosition, maxPosition;

    public WheelConfig() {
    }

    public void setName(String name) {
        this.name = name;
    }

    public String getName(String name) {
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

    public int getMinPostion() {
        return this.minPosition;
    }

    public void setMaxPosition(int maxPosition) {
        this.maxPosition = maxPosition;
    }

    public int getMaxPostion() {
        return this.maxPosition;
    }

    @Override
    public String toString() {
        return "Name: " + this.getName() + ", " +
            "ID: " + this.getId() + ", " +
            "InitPos: " + this.getInitPosition() + ", " +
            "MinPos: " + this.getMinPosition() + ", " +
            "MaxPos: " + this.getMaxPosition() + ".";
    }
}
