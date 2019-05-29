package com.slemenik.lidar.reconstruction.mountains;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import static com.slemenik.lidar.reconstruction.buildings.ShpController.getBoundsFromFilename;

public class MountainController {

    public double minX, maxX, minY, maxY;
    public double pointsSpace;

    public List<double[]> points2Insert = new ArrayList<>();
    public double[][] thirdDimInfo;

    public MountainController(double[][] arr, double pointsSpace) {
        setBounds(arr);
        this.pointsSpace = pointsSpace;
    }

    public MountainController(){}

    public void readAscFile(String fileName) {

        try {
            Scanner scanner = new Scanner(new File(fileName));
            while (scanner.hasNextLine()) {
                String line = scanner.nextLine();
                String coordinates[] = line.split(";");
                double x = Double.parseDouble(coordinates[0]);
                double y = Double.parseDouble(coordinates[1]);
                double z = Double.parseDouble(coordinates[2]);
                this.points2Insert.add(new double[]{x, y, z});

            }
            scanner.close();
        } catch (Exception e) {
            e.printStackTrace();
        }

    }

    public void parseFolder(String folderPath, String lazFilename) {
        this.points2Insert = new ArrayList<>();
        File fileList[] = new File(folderPath).listFiles();
        for(File file : fileList) {
            System.out.println("File z imenom:" + file.getName());
            try {
                Scanner scanner = new Scanner(file);
                while (scanner.hasNextLine()) {
                    String line = scanner.nextLine();
                    String coordinates[] = line.split("\\s+");
                    double x = Double.parseDouble(coordinates[0]);
                    double y = Double.parseDouble(coordinates[1]);
                    double z = Double.parseDouble(coordinates[2]);
                    int bounds[] = getBoundsFromFilename(lazFilename);

                    if (bounds[0] <= x && bounds[1] <= y && bounds[2] >= x && bounds[3] >= y) {
                        this.points2Insert.add(new double[]{x, y, z});
                    }
                    if (points2Insert.size() > 16 * 1000000) {
                        return;
                    }
                }
                scanner.close();
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            }
            System.out.println("trenutno tcck:" + points2Insert.size());
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

    private static int point2Index(double coordinate, double min, double pointSpace) {
        return (int) ((coordinate - min) / pointSpace);
    }

    private static double index2Point(int x, double min, double pointSpace) {
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

    /*each field is true if point exists*/
    public boolean[][] getBooleanPointField(double[][]  arr) {

        System.out.println("method getBooleanPointField()");
//        double[] bounds = getBounds(arr);
//        double minX = bounds[0];
//        double maxX = bounds[1];
//        double minY = bounds[2];
//        double maxY = bounds[3];

        boolean[][] field = initField(minX, maxX, minY, maxY, pointsSpace);
        this.thirdDimInfo = new double[field.length][field[0].length];

        System.out.println(field.length);
        System.out.println(field[0].length);

        int indexX = 0;
        int indexY = 0;
        double temp = 0;
        try {
            for (double[] arrEl : arr) {
                temp = arrEl[1]; //TODO temp - zbriši temp, arrEl je hardcodiran, naredi da nebo
                indexX = Integer.min(point2Index(arrEl[1], minX, pointsSpace), field.length - 1);
                indexY = Integer.min(point2Index(arrEl[2], minY, pointsSpace), field[0].length - 1);
                field[indexX][indexY] = true; //points exists
                thirdDimInfo[indexX][indexY] = thirdDimInfo[indexX][indexY] == 0 ? arrEl[0] : (arrEl[0]+thirdDimInfo[indexX][indexY])/2; //average
            }
        } catch (Exception e) {
            System.out.println("error");
            System.out.println(indexX);
            System.out.println(indexY);
            System.out.println(temp);
        }

        return field;

//        return getPointsFromFieldArray(field, minX, minY, pointsSpace);
    }

    public List<double[]> fillHoles(double[][]  arr) {
        System.out.println("method fillHoles()");
        boolean[][] fieldAllPoints = getBooleanPointField(arr);
        boolean[][] boundaryField = getBoundaryField(fieldAllPoints);

        List<int[]> pointsToInsert = new ArrayList<>();

       for (int i = 0; i< fieldAllPoints.length; i++) {
           for (int j = 0; j < fieldAllPoints[i].length; j++) {
               if (fieldAllPoints[i][j]) { //point already exists, ignore it
                   continue;
               } else {
                   //we check if it is inside boundary - it has a boundary point somewhere bellow, above, left AND right

                   //check up
                   int tempIndex = j;
                   boolean boundaryFound = false;
                   while (tempIndex < fieldAllPoints[0].length) {
                       if (boundaryField[i][tempIndex]) { //we have reached the boundary point
                           boundaryFound = true;
                           break;
                       }
                       tempIndex++;
                   }
                   if (!boundaryFound) {// point it outside boundary, we ignore it
                       continue;
                   }

                   //check down
                   tempIndex = j;
                   boundaryFound = false;
                   while (tempIndex >= 0 ) {
                       if (boundaryField[i][tempIndex]) {
                           boundaryFound = true; //we have reached the boundary point
                           break;
                       }
                       tempIndex--;
                   }
                   if (!boundaryFound) {// point it outside boundary, we ignore it
                       continue;
                   }


                   //check right
                   tempIndex = i;
                   boundaryFound = false;
                   while (tempIndex < fieldAllPoints.length ) {
                       if (boundaryField[tempIndex][j]) {
                           boundaryFound = true; //we have reached the boundary point
                           break;
                       }
                       tempIndex++;
                   }
                   if (!boundaryFound) {// point it outside boundary, we ignore it
                       continue;
                   }

                   //check left
                   tempIndex = i;
                   boundaryFound = false;
                   while (tempIndex >= 0 ) {
                       if (boundaryField[tempIndex][j]) {
                           boundaryFound = true; //we have reached the boundary point
                           break;
                       }
                       tempIndex--;
                   }
                   if (!boundaryFound) {// point it outside boundary, we ignore it
                       continue;
                   }

                   //it is inside boundary, we add it
                   pointsToInsert.add(new int[]{i, j});
               }
           } //end for j
       } // end for i

        return getPointsFromFieldList(pointsToInsert);
//        return getPointsFromFieldArray(fieldAllPoints, false);
    }


//        PrintStream printStream;

//        PrintStream ps = new PrintStream(System.out);
//        q.print(ps);
//        ps.flush();

//        System.out.println(q.size());
//        List<QuadTree<Integer>.CoordHolder> a = q.findAll( 137782.030 , 2761.990, 137783.340, 2762.980);
//        System.out.println(a.size());
//        for (QuadTree<Integer>.CoordHolder item : a) {
//            System.out.println(item.x);
//            System.out.println(item.y);
//            System.out.println(item.o);
//            System.out.println(item.depth());
//            System.out.println(item.quad);
//            System.out.println("_____________________-");
//
//        }





//        if (LP != null) q.LIST_PROVIDER = LP;
//        MAX = 1;
//        DYNAMIC = 0;
//        BUCKET_EXP = 0;
//        q.LEAF_MAX_OBJECTS = MAX;
//        if (DYNAMIC > 0)
//            q.DYNAMIC_MAX_OBJECTS = true;
//        q.MAX_OBJ_TARGET_EXPONENT = BUCKET_EXP;
//
//        for(int i = 0; i < nItems; i++)
//        {
//            q.place(Math.round(Math.random()*1000) + (windowShift * i / (double)nItems),
//                    Math.round(Math.random()*1000), i);
//        }
//        return q;

//

    public List<double[]> getPointsFromFieldArray(boolean[][] field, boolean writeWhenBoolean) {
        List<double[]> result = new ArrayList<>();
        for (int x = 0; x<field.length; x++) {
            double newX = index2Point(x, minX, pointsSpace);
            for (int y = 0; y<field[0].length; y++) {
                double newY = index2Point(y, minY, pointsSpace);
                if (field[x][y] == writeWhenBoolean) {
                    result.add(new double[]{ 410537.0, newX, newY});//temp, because x = 0, y = x, z = y
                }

            }
        }
        return result;
    }

    public static double getAverageThirdDim(double [][] thirdDimInfo, int indexX, int indexY) {
        //we dont need to check bounds, because this point cannot be boundary point
        double averageSum = 0;
        int count = 0;
        for (int i = indexX-1; i<= indexX+1; i++) {
            for (int j = indexY-1; j<=indexY+1; j++) {
                if (!(i == indexX && j == indexY) && thirdDimInfo[i][j] != 0) {
                    averageSum += thirdDimInfo[i][j];
                    count++;
                }
            }
        }
        return count == 0 ? -1 : averageSum/count;

    }

    public List<double[]> getPointsFromFieldList(List<int[]> fieldsWithPointList) {
        System.out.println("method getPointsFromFieldList()");
        List<double[]> result = new ArrayList<>();
        int i = 0;
        int requiredSize = fieldsWithPointList.size();
        while(result.size() < requiredSize) {
            int[] fieldIndex = fieldsWithPointList.get(i);
            i++;

            int indexX = fieldIndex[0];
            int indexY = fieldIndex[1];
            double newX = index2Point(indexX, minX, pointsSpace);
            double newY = index2Point(indexY, minY, pointsSpace);
            double newTemp = getAverageThirdDim(thirdDimInfo, indexX, indexY);

            if (newTemp == -1) { //no average found, we will calculate later
                fieldsWithPointList.add(fieldIndex);
            } else {
                thirdDimInfo[indexX][indexY] = newTemp;
                result.add(new double[]{newTemp, newX, newY});//temp, because x = 0, y = x, z = y
            }
            System.out.println("result.size() is " + result.size() + ", must be " + requiredSize);

        }
        return result;
    }


    public boolean [][] getBoundaryField(boolean [][] field) { //growth distance algorithm

        int K = 2;
        boolean [][] newField = new boolean[field.length][field[0].length];

        int firstX = -1;
        int firstY = -1;

        //get first point
        for (int i = 0; i<field.length; i++) {
            for (int j = 0; j < field[0].length; j++) {
                if (field[i][j]) {
                    firstX = i;
                    firstY = j;
                    break;
                }
            }
            if (firstX > -1 && firstY > -1) {
                break;
            }
        }

        int pX = firstX;
        int pY = firstY;

        int nextX = -1;
        int nextY = -1;
        int prevX = -1;
        int prevY = -1;

        boolean firstRound = true;
        int count = 0;

        boolean stop = false;
        while (!stop) {
            boolean find = stop;
            int i = 1;
            while (i <= K && !find) {

                List<int[]> rn = RN8(pX, pY, prevX, prevY, i, field.length, field[0].length);
                i++; //?
                for (int[] q : rn) {
                    int qX = q[0];
                    int qY = q[1];
                    if (!field[qX][qY]) {
                        continue; //current field are not a point, but are empty, so we skip it
                    }
                    if (is8NeighborhoodEmpty(field, qX, qY)) {
                         prevX = pX;
                         prevY = pY;

                         nextX = qX;
                         nextY = qY;

                         find = true;
                         break; //break for RN
                    } //end if
                } //end for
            } //end while
            if (pX == firstX && pY == firstY && !firstRound) {
                stop = true;
            } else {
                pX = nextX;
                pY = nextY;
                newField[pX][pY] = true;
                firstRound = false;
                count++;
            }
        }// end while
        System.out.println("count " + count);
        return newField;
    }

    private static boolean is4NeighborhoodEmpty(boolean [][] field, int x, int y) {
        return (x + 1 >= field.length
                || x - 1 < 0
                || y + 1 >= field[0].length
                || y - 1 < 0 // if index out of bounds, it is boundary
                || !field[x + 1][y])
                || !field[x - 1][y]
                || !field[x][y + 1]
                || !field[x][y - 1];

    }

    private static boolean is8NeighborhoodEmpty(boolean [][] field, int x, int y) {
        return is4NeighborhoodEmpty( field, x, y) ||
                (x + 1 < field.length && y + 1 < field[0].length && !field[x + 1][y+1])
                || (x - 1 >= 0 && y - 1 >= 0 && !field[x - 1][y-1])
                || (x + 1 < field.length && y - 1 >= 0 && !field[x+1][y-1])
                || (x - 1 >= 0 && y + 1 < field[0].length && !field[x-1][y+1]);

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

    public static List<int[]> RN8(int pX, int pY, int prevX, int prevY, int i, int lengthX, int lengthY) {
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

        int firstX = currX;
        int firstY = currY;

        while (result.size() < 3 || (firstX != currX || firstY != currY)) { // The  k-ring  neighborhood  of  p  contains  8k points

            if (pX == currX && pY == currY && result.size() > 0) {
                result.remove(result.size()-1);
                //continue; // somehow we are on the edge and we got current p in our k-ring, so we remove it
            }

            if (direction == Direction.RIGHT) {
                if (currX + 1 > pX + i || currX + 1 >= lengthX) {
                    direction = Direction.DOWN;
                } else {
                    currX++;
                    result.add(new int[]{currX, currY});
                }
            } else if (direction == Direction.DOWN){
                if (currY - 1 < pY - i || currY - 1 < 0) {
                    direction = Direction.LEFT;
                } else {
                    currY--;
                    result.add(new int[]{currX, currY});
                }
            }
            else if (direction == Direction.LEFT) {
                if (currX - 1 < pX - i || currX - 1 < 0) {
                    direction = Direction.UP;
                } else {
                    currX--;
                    result.add(new int[]{currX, currY});
                }
            } else if(direction == Direction.UP) {
                if (currY + 1 > pY + i || currY + 1 >= lengthY) {
                    direction = Direction.RIGHT;
                } else {
                    currY++;
                    result.add(new int[]{currX, currY});
                }
            }
        }
        return result;
    }



}
