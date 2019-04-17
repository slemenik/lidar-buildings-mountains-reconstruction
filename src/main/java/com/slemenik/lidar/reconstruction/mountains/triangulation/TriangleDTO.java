package com.slemenik.lidar.reconstruction.mountains.triangulation;

import java.util.Arrays;

public class TriangleDTO {

    private PointDTO point1;
    private PointDTO point2;
    private PointDTO point3;
    private EdgeDTO edge1;
    private EdgeDTO edge2;
    private EdgeDTO edge3;
    private int id;

    public TriangleDTO(PointDTO point1, PointDTO point2, PointDTO point3, int id)  {
        this.setId(id);

        //point are ALWAYS from lowest to highest
        PointDTO[] pointsArray = new PointDTO[]{point1, point2, point3};
        Arrays.sort(pointsArray);
        this.setPoint1(pointsArray[0]);
        this.setPoint2(pointsArray[1]);
        this.setPoint3(pointsArray[2]);

        this.setEdge1(new EdgeDTO(point1, point2, this));
        this.setEdge2(new EdgeDTO(point2, point3, this));
        this.setEdge3(new EdgeDTO(point3, point1, this));
    }

    @Override
    public boolean equals(Object obj) {
        TriangleDTO tr = (TriangleDTO) obj;
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

    public PointDTO getPoint1() {
        return point1;
    }

    public void setPoint1(PointDTO point1) {
        this.point1 = point1;
    }

    public PointDTO getPoint2() {
        return point2;
    }

    public void setPoint2(PointDTO point2) {
        this.point2 = point2;
    }

    public PointDTO getPoint3() {
        return point3;
    }

    public void setPoint3(PointDTO point3) {
        this.point3 = point3;
    }

    public EdgeDTO getEdge1() {
        return edge1;
    }

    public void setEdge1(EdgeDTO edge1) {
        this.edge1 = edge1;
    }

    public EdgeDTO getEdge2() {
        return edge2;
    }

    public void setEdge2(EdgeDTO edge2) {
        this.edge2 = edge2;
    }

    public EdgeDTO getEdge3() {
        return edge3;
    }

    public void setEdge3(EdgeDTO edge3) {
        this.edge3 = edge3;
    }

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }
}
