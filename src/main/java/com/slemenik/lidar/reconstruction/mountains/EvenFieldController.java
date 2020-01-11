package com.slemenik.lidar.reconstruction.mountains;

import java.sql.Time;
import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import java.util.stream.Stream;

//import org.apache.commons.math3.a

import com.slemenik.lidar.reconstruction.main.HelperClass;
import com.slemenik.lidar.reconstruction.main.Main;
import com.slemenik.lidar.reconstruction.main.TimeKeeper;
import com.slemenik.lidar.reconstruction.mountains.InterpolationController.Interpolation;
import org.apache.commons.lang3.ArrayUtils;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

public class EvenFieldController {

    public double minX, maxX, minY, maxY;
    public double pointsSpace;
    private Interpolation interpolation;
    //public InterpolationController.Interpolation interpolation = Interpolation.AVERAGE_8N;

    public List<double[]> points2Insert = new ArrayList<>();
    public double[][] thirdDimInfo; //todo namesto tega hrani raje tabelo objektov, kjer so vse info, tudi npr katere
                                    //todo vse double točke priapdajo specifičnemu polju, potem nebomo rabili v
                                    //todo getBooleanPointField() povprečiti thirdDimInfo vrednosti

//    public EvenFieldController(double[][] arr, double pointsSpace) { //todo remove in the future
////        setBounds(arr);
////        this.pointsSpace = pointsSpace;
////    }

    public EvenFieldController(double minX, double maxX, double minY, double maxY, double pointsSpace) {
        this.minX = minX;
        this.maxX = maxX;
        this.minY = minY;
        this.maxY = maxY;
        this.pointsSpace = pointsSpace;
        try {
            this.interpolation = Interpolation.valueOf(Main.INTERPOLATION_STRING);
        } catch (Exception e) {
            System.out.println("Specified interpolation (" + Main.INTERPOLATION_STRING + ") is not defined. Setting default interpolation to SPLINE");
            this.interpolation = Interpolation.SPLINE;
        }

}



    public static boolean[][] initField(double minX, double maxX, double minY, double maxY, double pointSpace) {


        double a = maxX - minX;
//        System.out.println(maxY - minY);
        int dimX = (int) ((maxX - minX) / pointSpace);
        int dimY = (int) ((maxY - minY) / pointSpace);
//        System.out.println(dimX);
//        System.out.println(dimY);
        boolean[][] field = new boolean[dimX][dimY];
        return field;
    }

    public static int point2Index(double coordinate, double min, double pointSpace) {//temp spremeni v private
//        return (int) ((coordinate - min) / pointSpace);
        return (int) (Math.round((coordinate - min) / pointSpace));
    }

    public static double index2Point(int x, double min, double pointSpace) {//temp spremeni v private
        return ((double) x) * pointSpace + min;
    }

    public void setBounds(double[][] arr) {

        int i = 0;
//        QuadTree<Integer> q = new QuadTree<>();
        double minX = Integer.MAX_VALUE;
        double maxX = 0;
        double minY = Integer.MAX_VALUE;
        double maxY = 0;
        for (double[] arrEl: arr) {
            //arr[i][0] = 0; //temp
//            System.out.println(String.format("%f %f %f", arrEl[0], arrEl[1], arrEl[2] ));
//            q.place(arr[i][1],arr[i][2], i);


            maxX = Double.max(maxX, arr[i][1] );
            minX = Double.min(minX, arr[i][1] );
            maxY = Double.max(maxY, arr[i][2] );
            minY = Double.min(minY, arr[i][2] );
            i++;
        }

        System.out.println("minX " + minX);
        System.out.println("maxX " + maxX);
        System.out.println("minY " + minY);
        System.out.println("maxY " + maxY);

        this.maxX = maxX;
        this.minX = minX;
        this.maxY = maxY;
        this.minY = minY;

//        return new double[]{minX, maxX, minY, maxX};

    }

    public boolean[][] getBooleanPointField(double[][] pointArray) {
        return getBooleanPointField(Arrays.asList(pointArray).stream().map(d -> new Point3d(d[0], d[1], d[2])).collect(Collectors.toSet()));
    }


    /*each field is true if point exists*/
    public boolean[][] getBooleanPointField(Set<Point3d> pointList) {

//        System.out.println("method getBooleanPointField()");
//        double[] bounds = getBounds(pointList);
//        double minX = bounds[0];
//        double maxX = bounds[1];
//        double minY = bounds[2];
//        double maxY = bounds[3];

        boolean[][] field = initField(minX, maxX, minY, maxY, pointsSpace);
        this.thirdDimInfo = new double[field.length][field[0].length];

//        System.out.println(field.length);
//        System.out.println(field[0].length);

        int indexX = 0;
        int indexY = 0;
//        double temp = 0;
        try {
            for (Point3d point : pointList) {

                indexX = Integer.min(point2Index(point.x, minX, pointsSpace), field.length - 1);
                indexY = Integer.min(point2Index(point.y, minY, pointsSpace), field[0].length - 1);
                field[indexX][indexY] = true; //points exists
                thirdDimInfo[indexX][indexY] = thirdDimInfo[indexX][indexY] == 0 ? point.z : Double.max(point.z, thirdDimInfo[indexX][indexY]); // todo poglej a je kul da je povprečje vseh točk na istmu indexu

//                thirdDimInfo[indexX][indexY] = thirdDimInfo[indexX][indexY] == 0 ? arrEl[2] : (arrEl[2]+thirdDimInfo[indexX][indexY])/2; //average

//                if (HelperClass.isBetween(arrEl[0], 410839.680, 410839.699) && HelperClass.isBetween(arrEl[1], 137211.260, 137211.280)) {
//                    int tempa = 5;
//                }
            }
        } catch (Exception e) {
            System.out.println("error");
            System.out.println(indexX);
            System.out.println(indexY);
            System.out.println(e);
        }

        return field;

//        return getPointsFromFieldArray(field, minX, minY, pointsSpace);
    }

    public List<Point3d> fillHoles(SortedSet<Point3d>  pointList) {
//        System.out.println("method fillHoles()");
        if (pointList.size() == 0) {
            System.out.println("fillHoles() - empty array param. return empty list");
            return new ArrayList<>();
        }
        boolean[][] fieldAllPoints = getBooleanPointField(pointList); //boolean field of projection where value==true if point exists
        TimeKeeper.projectionEndTime();
        //HelperClass.createFieldPointFile(fieldAllPoints, minX, minY, pointsSpace);
        if (MountainController.debug)
            HelperClass.createFieldPointFile(fieldAllPoints);
        TimeKeeper.borderStartTime();
        boolean[][] boundaryField = getBoundaryField(fieldAllPoints); //boolean field, subset of fieldAllPoints, only boundary points are true
        TimeKeeper.borderEndTime();
        //HelperClass.createFieldPointFile(boundaryField, minX, minY, pointsSpace);
        if (MountainController.debug)
            HelperClass.createFieldPointFile(boundaryField);
            int tempa=5;
            //System.exit(-1111);
//        }


        List<int[]> pointsToInsert = new ArrayList<>(); //list of indices, index x and y of fieldAllPoints represent new point to insert

        TimeKeeper.innerHolesStartTime();
       for (int i = 0; i< fieldAllPoints.length; i++) {
           boolean insideBorder = false;
           List<int[]> pointsToInsertPerRow = new ArrayList<>();
           for (int j = 0; j < fieldAllPoints[i].length; j++) {
               if (boundaryField[i][j]) { // point is boundary
                   if (pointsToInsertPerRow.size() > 0 && insideBorder) { //if we had written some points and we were previously inside border
                       pointsToInsert.addAll(pointsToInsertPerRow); //we write all indexes representing new (not yet added) points
                   }
                   insideBorder = false; //we are not inside boundary anymore
                   pointsToInsertPerRow.clear(); //delete points we created from last border point to this one
               } else if (!fieldAllPoints[i][j]) { //point is not boundary and does not yet exists at this index
                       pointsToInsertPerRow.add(new int[]{i,j}); //we add it to temporary list, we don't know yet if points are inside border
               } else {
                   insideBorder = true; //point is not boundary, and already exists at current index - we are definitely inside border,
                                        // otherwise there couldn't be a point already here. Every point added from last border point to next
                                        // will be added to main list, when we hit the next border point
               }
           } //end for j
       } // end for i
        TimeKeeper.innerHolesEndTime();
//        if (pointList.size() == 2264018) { //temp
//        HelperClass.createFieldPointFile(pointsToInsert, minX, minY, pointsSpace);
//            HelperClass.createFieldPointFile(pointsToInsert);
//        }
        return getPointsFromFieldList(pointsToInsert);
//        return getPointsFromFieldArray(fieldAllPoints, false);
    }

    //usage: testBoundary() , testMountainGrid3d
//    public List<double[]> getPointsFromFieldArray(boolean[][] field, boolean writeWhenBoolean) {
//        List<double[]> result = new ArrayList<>();
//        for (int x = 0; x<field.length; x++) {
//            double newX = index2Point(x, minX, pointsSpace);
//            for (int y = 0; y<field[0].length; y++) {
//                double newY = index2Point(y, minY, pointsSpace);
//                if (field[x][y] == writeWhenBoolean) {
//                    double newTemp = InterpolationController.getThirdDim(thirdDimInfo, x, y, Main.INTERPOLATION);
//                    if (newTemp != -2) {
//                        result.add(new double[]{ newTemp, newX, newY});//temp, because x = 0, y = x, z = y
//                    }
//                }
//
//            }
//        }
//        return result;
//    }



    public List<Point3d> getPointsFromFieldList(List<int[]> fieldsWithPointList) {
//        System.out.println("method getPointsFromFieldList()");
        TimeKeeper.interpolationStartTime();
        List<Point3d> result = new ArrayList<>();
        int i = 0;
        int requiredSize = fieldsWithPointList.size();
        while(result.size() < requiredSize) {
            int[] fieldIndex = fieldsWithPointList.get(i);

            int indexX = fieldIndex[0];
            int indexY = fieldIndex[1];
            double newX = index2Point(indexX, minX, pointsSpace);
            double newY = index2Point(indexY, minY, pointsSpace);
//            if (indexX == 512 && indexY == 167) {
//                int tempa=5;
//            }
//            if (HelperClass.isBetween(newX, 410839.680, 410839.699) && HelperClass.isBetween(newY, 137211.260, 137211.280)) {
//                int tempa = 5;
//            }
            double newTemp = InterpolationController.getThirdDim(thirdDimInfo, indexX, indexY, interpolation);
//todo tukaj se pri interpolaciji večkrat kličejo ene in iste točke, popravi, npr ko je indexX skos isti, se potem na podlagi tega iste interpolacije računajo
            if (newTemp == -1) { //no average found, we will calculate later
//                System.out.println("no average found");
                fieldsWithPointList.add(fieldIndex);
            }else if(newTemp == -2) {
                requiredSize--;
//                fieldsWithPointList.remove(i);
//                i--; //we subtract it and moment later we add it, so we get next point correctly
            } else {
                if (!ArrayUtils.contains(new Interpolation[]{Interpolation.QUADRATIC_NEAREST, Interpolation.SPLINE_OLD}, interpolation)) { //todo explore why
                    thirdDimInfo[indexX][indexY] = newTemp; // sprotno popravljanje oz dopolnjevanje tretje dimenzije
                }
                result.add(new Point3d (newX, newY, newTemp));//temp, because x = 0, y = x, z = y
            }
//            System.out.println("result.size() is " + result.size() + ", must be " + requiredSize);
            i++;
        }
        if (result.size() == 22656) {
            int temp = 24;
        }
        TimeKeeper.interpolationEndTime();
        return result;
    }

    public static boolean [][] getBoundaryField(boolean [][] field) { //growth distance algorithm
//        System.out.println("method getBoundaryField()");
        int K = 2;//temp
        boolean [][] newField = new boolean[field.length][field[0].length];

        Point2d pFirst = new Point2d(-1,-1);

        //get first point
        for (int i = 0; i<field.length; i++) {
            for (int j = 0; j < field[0].length; j++) {
                if (field[i][j]) {
                    pFirst.x = i;
                    pFirst.y = j;
                    break;
                }
            }
            if (pFirst.x > -1 && pFirst.y > -1) {
                break;
            }
        }
        Point2d p = pFirst;
        Point2d pNext = new Point2d(-1,-1);
        Point2d pPrev = new Point2d(-1,-1);;

        boolean firstRound = true;
        int count = 0;

        boolean stop = false;
        while (!stop) {
            boolean find = stop;
            int i = 1;
            while (/*i <= K temp &&*/ !find) {

                List<int[]> rn = RN8(p, pPrev, i, field.length, field[0].length);
                if (rn == null) {
                    HelperClass.createFieldPointFile(field);
                    HelperClass.printLine("", "impossible rn == null");
                }
//                List<int[]> rn = RN8(p, pPrev, i, field.length, field[0].length);
                i++; //?
                for (int[] qArray : rn) {
                    Point2d q = new Point2d(qArray[0], qArray[1]);
                    if (!field[(int)q.x][(int)q.y]) {
                        continue; //at current position there is no point, so this is not q
                    }
                    if (is8NeighborhoodEmpty(field, (int)q.x, (int)q.y, true)) {
                         pPrev = p;
                         pNext = q;
                         find = true;
                         break; //break for RN
                    } else {
                        int atemp = 5;
                        //todo do tega nikoli ne pride, ergo je IF za bv
                        //+ todo poglej kdaj pride do neskončne zanke in kako lahko s K in i vplivaš
                    }//end if
                } //end for
            } //end while
            if (p.equals(pFirst) && !firstRound) {
                stop = true;
            } else {
                p = pNext;
                try {
                    newField[(int)p.x][(int)p.y] = true;
                } catch (ArrayIndexOutOfBoundsException e) {
                    HelperClass.printLine(", ", p, stop, pFirst);
                    HelperClass.createFieldPointFile(field);
                    newField[(int)p.x][(int)p.y] = true;
                }
                firstRound = false;
                count++;
                if (count > field.length * field[0].length) { //we basically went over whole field, still did not find solution
                    //K++;
                    //stop = true;
                    count = 0;
//                    HelperClass.createFieldPointFile(newField);
                    return newField;

                }
                if (K == 100) {
                    HelperClass.createFieldPointFile(newField);
                    K++;
                }
            }
        }// end while
//        System.out.println("count " + count);
        return newField;
    }

//    public static boolean [][] growthClockWise(boolean [][] field) { //growth clock wise
//        int k = 2;//temp
//        boolean stop = false;
//        while (!stop) {
//            //computepPrevk
//            boolean find = false;
//            for (int[] q : RNk) {
//                if (find) {//todo ?
//                    break;
//                }
//                int qX = q[0];
//                int qY = q[1];
//                for (int i = 1; i<= k; i++) {
//                    q
//                }
//            } // end foreach RNk
//            if (pX == firstX && pY == firstY && !firstRound) {
//                stop = true;
//            }
//        } //end while stop
//    } //end function

    private static boolean is4NeighborhoodEmpty(boolean [][] field, int x, int y, boolean outsideBoundsIsEmpty) {
        return outsideBoundsIsEmpty ? (
                x + 1 >= field.length
                || x - 1 < 0
                || y + 1 >= field[0].length
                || y - 1 < 0
                || !field[x + 1][y]
                || !field[x - 1][y]
                || !field[x][y + 1]
                || !field[x][y - 1]
        ) : (
                x + 1 < field.length && !field[x + 1][y]
                || x - 1 >= 0 && !field[x - 1][y]
                || y + 1 < field[0].length && !field[x][y + 1]
                || y - 1 >= 0 && !field[x][y - 1]
        );

    }

    private static boolean is4NeighborhoodEmpty(boolean [][] field, int x, int y) {
        return is4NeighborhoodEmpty(field, x, y, true);
    }

    private static boolean is8NeighborhoodEmpty(boolean [][] field, int x, int y, boolean outsideBoundsIsEmpty) {
        return is4NeighborhoodEmpty( field, x, y, outsideBoundsIsEmpty) || // if outsideBoundsIsEmpty == true && is4NeighborhoodEmpty == false, it defenetly is NOT out of bounds
                (x + 1 < field.length && y + 1 < field[0].length && !field[x + 1][y+1])
                || (x - 1 >= 0 && y - 1 >= 0 && !field[x - 1][y-1])
                || (x + 1 < field.length && y - 1 >= 0 && !field[x+1][y-1])
                || (x - 1 >= 0 && y + 1 < field[0].length && !field[x-1][y+1]);

    }

    private static boolean is8NeighborhoodEmpty(boolean [][] field, int x, int y) {
        return is4NeighborhoodEmpty(field, x, y, true);
    }

    private static List<int[]> RN4(int pX, int pY, int prevX, int prevY, int i) {
        return null;
    }

    private enum Direction {
        RIGHT,
        DOWN,
        LEFT,
        UP
    }

    private static List<int[]> RN8(Point2d p, Point2d pPrev, int i, int lengthX, int lengthY) {
        return RN8((int)p.x, (int)p.y, (int)pPrev.x, (int)pPrev.y, i, lengthX, lengthY);
    }

    /*return list of indices {[x,y], [x2,y2],... } that surround point px,py */
    public static List<int[]> RN8(int pX, int pY, int prevX, int prevY, int i, int lengthX, int lengthY) {
        if (i > lengthX && i > lengthY) {
            return null;
        }

        List<int[]> result = new ArrayList<>();
        if (prevX == -1 || prevY == -1) {
            prevX = Integer.max(pX - i, 0);
            prevY = Integer.max(pY - i, 0);
        }
//
        Direction direction;
        int currX, currY;

        if (prevY < pY && prevX == pX) { //south
            direction = Direction.LEFT;
            currX = pX;
            currY = pY-i;
        } else if (prevY < pY && prevX < pX) { //southwest
            direction = Direction.UP;
            currX = pX-i;
            currY = pY-i;
        } else if (prevY == pY && prevX < pX) { //west
            direction = Direction.UP;
            currX = pX-i;
            currY = pY;
        } else if (prevY > pY && prevX < pX) { //northwest
            direction = Direction.RIGHT;
            currX = pX-i;
            currY = pY+i;
        } else if (prevY > pY && prevX == pX) { //north
            direction = Direction.RIGHT;
            currX = pX;
            currY = pY+i;
        } else if (prevY > pY && prevX > pX) { //northeast
            direction = Direction.DOWN;
            currX = pX+i;
            currY = pY+i;
        } else if (prevY == pY && prevX > pX) { //east
            direction = Direction.DOWN;
            currX = pX+i;
            currY = pY;
        } else if (prevY < pY && prevX > pX) { //southeast
            direction = Direction.LEFT;
            currX = pX+i;
            currY = pY-i;
        } else if (prevX == pX && prevY == pY) {
            direction = Direction.UP;
            currX = pX;
            currY = pY;
        } else {

            System.out.println("impossible");
            System.out.println(String.format("prevX %d, prevY %d, px %d, py %d", prevX,  prevY, pX, pY));
            return null;
        }
        currX = HelperClass.getValueInsideBounds(currX, lengthX); //firstX must be between 0 and lengthX-1
        currY = HelperClass.getValueInsideBounds(currY, lengthY); //firstY must be between 0 and lengthY-1

        int firstX = currX;
        int firstY = currY;

        while (result.size() < 3 || (firstX != currX || firstY != currY)) { // The  k-ring  neighborhood  of  p  contains  8k points

            if (result.size() > 8 * i) {
                HelperClass.printLine("", "The  k-ring  neighborhood  of  p  contains  more than 8k points!!");
            }

            if (pX == currX && pY == currY && result.size() > 0) {
                result.remove(result.size()-1);
                //continue; // somehow we are on the edge and we got current p in our k-ring, so we remove it
            }

            if (direction == Direction.RIGHT) {
                if (currX + 1 > pX + i || currX + 1 >= lengthX) {
                    direction = Direction.DOWN;
                } else {
                    currX++;
                    if (currY < 0 || currX < 0) {
                        HelperClass.printLine("", "wt");
                    }
                    result.add(new int[]{currX, currY});
                }
            } else if (direction == Direction.DOWN){
                if (currY - 1 < pY - i || currY - 1 < 0) {
                    direction = Direction.LEFT;
                } else {
                    currY--;
                    if (currY < 0 || currX < 0) {
                        HelperClass.printLine("", "wt");
                    }
                    result.add(new int[]{currX, currY});
                }
            }
            else if (direction == Direction.LEFT) {
                if (currX - 1 < pX - i || currX - 1 < 0) {
                    direction = Direction.UP;
                } else {
                    currX--;
                    if (currY < 0 || currX < 0) {
                        HelperClass.printLine("", "wt");
                    }
                    result.add(new int[]{currX, currY});
                }
            } else if(direction == Direction.UP) {
                if (currY + 1 > pY + i || currY + 1 >= lengthY) {
                    direction = Direction.RIGHT;
                } else {
                    currY++;
                    if (currY < 0 || currX < 0) {
                        HelperClass.printLine("", "wt");
                    }
                    result.add(new int[]{currX, currY});
                }
            }
        }
        return result;
    }

    private static List<int[]> RN8alt(Point2d p, Point2d pPrev, int K, int lengthX, int lengthY) {
        List<int[]> ringIndicesList = new ArrayList<>();

        if (K > lengthX && K > lengthY) {
           return null;
        }

        if (pPrev.x == -1 || pPrev.y == -1) {
            pPrev.x = Integer.max((int)p.x - K, 0);
            pPrev.y = Integer.max((int)p.y - K, 0);
        }
        //calculate linear function of halfline p->pprev https://keisan.casio.com/exec/system/1223508685
        double k = (pPrev.y - p.y)/(pPrev.x - p.x);
        double n = ((pPrev.x * p.y) - (p.x * pPrev.y))/(pPrev.x - p.x);
        double functionY, functionX;
        int difference;
        int lowestDifference = Integer.MAX_VALUE;
        //set bounds of K ring
        int minX = Integer.max(0, (int) p.x - K);
        int maxX = Integer.min(lengthX-1, (int) p.x + K);
        int minY = Integer.max(0, (int) p.y - K);
        int maxY = Integer.min(lengthY-1 ,(int) p.y + K);

        //we start with lower left corner
        int ringX = minX;
        int ringY = minY;



        //move from lower left corner to upper left
        while (ringY < maxY) {
            if (!(ringX == (int)p.x && ringY == (int) p.y)) { //if current ring coordinate is not current point
                functionY = (k * ringX) + n;
                difference = Math.abs(ringY - (int) functionY);
                lowestDifference = Integer.min(difference, lowestDifference);
                ringIndicesList.add(new int[]{ringX, ringY, difference});
            }
            ringY++;
        }

        //move from upper left to upper right
        while (ringX < maxX) {
            if (!(ringX == (int)p.x && ringY == (int) p.y)) { //if current ring coordinate is not current point
                functionY = (k * ringX) + n;
                difference = Math.abs(ringY - (int) functionY);
                lowestDifference = Integer.min(difference, lowestDifference);
                ringIndicesList.add(new int[]{ringX, ringY, difference});
            }
            ringX++;
        }

        //move from upper right to lower right
        while (ringY > minY) {
            if (!(ringX == (int)p.x && ringY == (int) p.y)) { //if current ring coordinate is not current point
                functionY = (k * ringX) + n;
                difference = Math.abs(ringY - (int) functionY);
                lowestDifference = Integer.min(difference, lowestDifference);
                ringIndicesList.add(new int[]{ringX, ringY, difference});
            }
            ringY--;
        }

        //move from lower right to lower left (starting point)
        while (ringX > minX) {
            if (!(ringX == (int)p.x && ringY == (int) p.y)) { //if current ring coordinate is not current point
                functionY = (k * ringX) + n;
                difference = Math.abs(ringY - (int) functionY);
                lowestDifference = Integer.min(difference, lowestDifference);
                ringIndicesList.add(new int[]{ringX, ringY, difference});
            }
            ringX--;
        }

        //we shift the list so it starts ith pPrev
        int finalLowestDifference = lowestDifference;
        int indexOfPointOnHalfLine = IntStream.range(0, ringIndicesList.size()).filter(i -> {
            int[] ringIndex = ringIndicesList.get(i);
            return ringIndex[2] == finalLowestDifference //it lies on halfline p->pPrev
                    && (ringIndex[0] < p.x && pPrev.x < p.x || ringIndex[0] > p.x && pPrev.x > p.x ) //both ring X and pPrev X are on the same side of p.X (in direction from p to pPrev)
                    && (ringIndex[1] < p.y && pPrev.y < p.y || ringIndex[1] > p.y && pPrev.y > p.y ); //both ring Y and pPrev Y are on the same side of p.Y  (in direction from p to pPrev)
        }).findFirst().orElse(0);

        return Stream.concat(
                ringIndicesList.subList(indexOfPointOnHalfLine, ringIndicesList.size()).stream(),
                ringIndicesList.subList(0, indexOfPointOnHalfLine).stream()
        ).collect(Collectors.toList());

    }


}
