package com.slemenik.lidar.reconstruction.mountains.triangulation.model;

import java.util.Arrays;

public class Triangle {

    private Point point1;
    private Point point2;
    private Point point3;
    private Edge edge1;
    private Edge edge2;
    private Edge edge3;
    private int id;

    public Triangle(Point point1, Point point2, Point point3, int id)  {
        this.setId(id);

        //point are ALWAYS from lowest to highest
        Point[] pointsArray = new Point[]{point1, point2, point3};
        Arrays.sort(pointsArray);
        this.setPoint1(pointsArray[0]);
        this.setPoint2(pointsArray[1]);
        this.setPoint3(pointsArray[2]);

        this.setEdge1(new Edge(point1, point2, this));
        this.setEdge2(new Edge(point2, point3, this));
        this.setEdge3(new Edge(point3, point1, this));
    }

    @Override
    public boolean equals(Object obj) {
        Triangle tr = (Triangle) obj;
        return (tr.getPoint1().equals(this.getPoint1()) && tr.getPoint2().equals(this.getPoint2()) && tr.getPoint3().equals(this.getPoint3()));
    }

    @Override
    public int hashCode() {
        return getId();
    }

    @Override
    public String toString() {

        return "T." + getPoint1() +","+ getPoint2() + ","+ getPoint3();// + ": points(" + this.point1 + "; " + this.point2 + "; " + point3;
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

    public Point getPoint3() {
        return point3;
    }

    public void setPoint3(Point point3) {
        this.point3 = point3;
    }

    public Edge getEdge1() {
        return edge1;
    }

    public void setEdge1(Edge edge1) {
        this.edge1 = edge1;
    }

    public Edge getEdge2() {
        return edge2;
    }

    public void setEdge2(Edge edge2) {
        this.edge2 = edge2;
    }

    public Edge getEdge3() {
        return edge3;
    }

    public void setEdge3(Edge edge3) {
        this.edge3 = edge3;
    }

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }
}
