package com.slemenik.lidar.reconstruction.mountains.triangulation;

public class EdgeDTO {

    private PointDTO point1;
    private PointDTO point2;
    private TriangleDTO parentTriangle;

    public EdgeDTO(PointDTO p1, PointDTO point2, TriangleDTO parentTriangle){
        this.setParentTriangle(parentTriangle);
        //from lowest PointDTO.id to highest
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

        return ((EdgeDTO) obj).getPoint1().equals(this.getPoint1()) && ((EdgeDTO) obj).getPoint2().equals(this.getPoint2());
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

    public TriangleDTO getParentTriangle() {
        return parentTriangle;
    }

    public void setParentTriangle(TriangleDTO parentTriangle) {
        this.parentTriangle = parentTriangle;
    }
}
