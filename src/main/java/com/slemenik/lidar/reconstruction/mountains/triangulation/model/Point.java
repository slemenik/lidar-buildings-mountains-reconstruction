package com.slemenik.lidar.reconstruction.mountains.triangulation.model;

public class Point implements Comparable<Point> {

    private double x;
    private double y;
    private double z;
    private int id;

    public Point(double x, double y, double z, int id) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.id = id;
    }

    @Override
    public boolean equals(Object obj) {
        Point p = (Point) obj;
        return (p.getX() == this.getX() && p.getY() == this.getY() && p.getZ() == this.getZ());
    }

    @Override
    public int hashCode() {
//            int hash = 7;
//            hash = 71 * hash + Double.valueOf(this.x).hashCode();
//            hash = 71 * hash + Double.valueOf(this.y).hashCode();
//            hash = 71 * hash + Double.valueOf(this.z).hashCode();
//            return hash;

        return getId();
    }

    @Override
    public String toString() {
        return "P" + getId();// + "|" + "x= " + this.x + ", y = " + this.y + ", z = " + this.z;
    }

    @Override
    public int compareTo(Point o) {
        if (this.getId() > o.getId()) {
            return 1;
        } else {
            return -1;
        }
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getZ() {
        return z;
    }

    public void setZ(double z) {
        this.z = z;
    }

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }
}
