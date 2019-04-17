package com.slemenik.lidar.reconstruction.mountains.triangulation.model;

public class Edge {

    private Point point1;
    private Point point2;
    private Triangle parentTriangle;

    public Edge (Point p1, Point point2, Triangle parentTriangle){
        this.setParentTriangle(parentTriangle);
        //from lowest Point.id to highest
        if (p1.getId() < point2.getId()){
            this.setPoint1(p1);
            this.setPoint2(point2);
        } else if (p1.getId() > point2.getId()){
            this.setPoint1(point2);
            this.setPoint2(p1);
        }else {
            try {
                System.out.println("Napaka pri tockah: " + p1 + point2);
                throw new Exception();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public boolean equals(Object obj) {

        return ((Edge) obj).getPoint1().equals(this.getPoint1()) && ((Edge) obj).getPoint2().equals(this.getPoint2());
    }

    @Override
    public int hashCode() {
        int hash = 7;
        hash = 71 * hash + this.getPoint1().hashCode();
        hash = 71 * hash + this.getPoint2().hashCode();
        return hash;

    }

    @Override
    public String toString() {

        return "E."+ getPoint1() +"," + getPoint2() + "(" + getParentTriangle() +")";
    }

    public Point getPoint1() {
        return point1;
    }

    public void setPoint1(Point point1) {
        this.point1 = point1;
    }

    public Point getPoint2() {
        return point2;
    }

    public void setPoint2(Point point2) {
        this.point2 = point2;
    }

    public Triangle getParentTriangle() {
        return parentTriangle;
    }

    public void setParentTriangle(Triangle parentTriangle) {
        this.parentTriangle = parentTriangle;
    }
}
