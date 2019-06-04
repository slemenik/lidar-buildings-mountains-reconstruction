package com.slemenik.lidar.reconstruction.mountains;

import org.apache.commons.lang3.ArrayUtils;
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

public class InterpolationController {

    private static double getAverageThirdDim(double [][] thirdDimInfo, int indexX, int indexY, int neighbourhood) {
        //this point cannot be boundary point, when neighbourhood == 1
        double averageSum = 0;
        int count = 0;
        for (int i = indexX-neighbourhood; i<= indexX+neighbourhood; i++) {
            for (int j = indexY-neighbourhood; j<=indexY+neighbourhood; j++) {
                try {
                    if (!(i == indexX && j == indexY) && thirdDimInfo[i][j] != 0) {
                        averageSum += thirdDimInfo[i][j];
                        count++;
                    }
                } catch (ArrayIndexOutOfBoundsException e) {
                    //do nothing, we hit boundary and we dont care
                }
            }
        }
        return count == 0 ? -1 : averageSum/count;
    }

    private static double getAverageThirdDim(double [][] thirdDimInfo, int indexX, int indexY) {
        return getAverageThirdDim( thirdDimInfo,  indexX,  indexY, 1);
    }

    private static double getAverage4NThirdDim(double [][] thirdDimInfo, int indexX, int indexY) {
        double averageSum = 0;
        int count = 0;
        if (thirdDimInfo[indexX+1][indexY] != 0) {
            averageSum += thirdDimInfo[indexX+1][indexY];
            count++;
        }
        if (thirdDimInfo[indexX-1][indexY] != 0) {
            averageSum += thirdDimInfo[indexX-1][indexY];
            count++;
        }
        if (thirdDimInfo[indexX][indexY+1] != 0) {
            averageSum += thirdDimInfo[indexX][indexY+1];
            count++;
        }
        if (thirdDimInfo[indexX][indexY-1] != 0) {
            averageSum += thirdDimInfo[indexX][indexY-1];
            count++;
        }
        return count == 0 ? -1 : averageSum/count;
    }

    private static double getNearestThirdDim(double [][] thirdDimInfo, int indexX, int indexY) {
        return getNearestThirdDim(thirdDimInfo,  indexX,  indexY,  false);
    }

    private static double getNearestThirdDim(double [][] thirdDimInfo, int indexX, int indexY, boolean randomNeighbour) {

        List<Double> list = new ArrayList<>();
        for (int i = indexX-1; i<= indexX+1; i++) {
            for (int j = indexY-1; j<=indexY+1; j++) {
                if (!(i == indexX && j == indexY) && thirdDimInfo[i][j] != 0) {
                    if (randomNeighbour) {
                        list.add(thirdDimInfo[i][j]);
                    } else {
                        return thirdDimInfo[i][j];
                    }
                }
            }
        }
        if (list.size()>0) {
            Random rand = new Random();
            return list.get(rand.nextInt(list.size()));
        } else {
            return -1;
        }
    }

    public enum Interpolation {
        NEAREST_N,
        NEAREST_N_RANDOM,
        AVERAGE_8N,
        AVERAGE_4N,
        CONSTANT,
        OWN_VALUE,
        AVERAGE_24N,
        BILINEAR,
        BIQUADRATIC,
        BIQUADRATIC_NEAREST,
        BICUBIC,
        SPLINE
    }

    static double getThirdDim(double[][] thirdDimInfo, int indexX, int indexY, Interpolation interpolation) {
        switch (interpolation) {
            case NEAREST_N:
                return getNearestThirdDim(thirdDimInfo, indexX, indexY);
            case NEAREST_N_RANDOM:
                return getNearestThirdDim(thirdDimInfo, indexX, indexY, true);
            case AVERAGE_4N:
                return getAverage4NThirdDim(thirdDimInfo, indexX, indexY);
            case AVERAGE_8N:
                return getAverageThirdDim(thirdDimInfo, indexX, indexY);
            case AVERAGE_24N:
                return getAverageThirdDim(thirdDimInfo, indexX, indexY,2 );
            case CONSTANT:
                return 1;
            case OWN_VALUE:
                return thirdDimInfo[indexX][indexY]; //only when point we point to write alredy existed
            case BILINEAR:
                return getBiLinearThirdDim(thirdDimInfo, indexX, indexY);
            case BIQUADRATIC_NEAREST:
                return getBiQuadraticThirdDim(thirdDimInfo, indexX, indexY, true);
            case BIQUADRATIC:
                return getBiQuadraticThirdDim(thirdDimInfo, indexX, indexY, false);
            case BICUBIC:
                return getBiCubicThirdDim(thirdDimInfo, indexX, indexY);
            case SPLINE:
                return getSplineThirdDim(thirdDimInfo, indexX, indexY);
            default:
                System.out.println("wrong interpolation param");
                return -1;
        }
    }

    private double getAverage16NThirdDim(double[][] thirdDimInfo, int indexX, int indexY) {
        return 0;
    }

    private static double getBiLinearThirdDim(double[][] thirdDimInfo, int indexX, int indexY) {

        int rightNeighbourIndex = indexX+1;
        while (thirdDimInfo[rightNeighbourIndex][indexY] <= 0) {
            rightNeighbourIndex++;
        }
        int leftNeighbourIndex = indexX-1;
        while (thirdDimInfo[leftNeighbourIndex][indexY] <= 0) {
            leftNeighbourIndex--;
        }
        int upperNeighbourIndex = indexY+1;
        while (thirdDimInfo[indexX][upperNeighbourIndex] <= 0) {
            upperNeighbourIndex++;
        }
        int bottomNeighbourIndex = indexY-1;
        while (thirdDimInfo[indexX][bottomNeighbourIndex] <= 0) {
            bottomNeighbourIndex--;
        }


        LinearInterpolator li = new LinearInterpolator();
        PolynomialSplineFunction functionX = li.interpolate(new double[]{leftNeighbourIndex, rightNeighbourIndex}, new double[]{thirdDimInfo[leftNeighbourIndex][indexY], thirdDimInfo[rightNeighbourIndex][indexY]});
        PolynomialSplineFunction functionY = li.interpolate(new double[]{bottomNeighbourIndex, upperNeighbourIndex}, new double[]{thirdDimInfo[indexX][bottomNeighbourIndex], thirdDimInfo[indexX][upperNeighbourIndex]});

        double valueX = functionX.value(indexX);
        double valueY = functionY.value(indexY);

//        PolynomialSplineFunction function = li.interpolate(new double[]{}, new double[]{valueX, valueY});
        return (valueX+valueY)/2;
//        InterpolationBilinear bilinear = new InterpolationBilinear();
//        bilinear.
//        return 0;

    }

    // http://mathonline.wikidot.com/deleted:quadratic-polynomial-interpolation
    private static double interpolateQuadratic(Double[] x, Double[] y, double value)  {

        if (x.length != y.length || x.length != 3 ) {
            System.out.println("Wrong parameter num");
            return -1;
        } else {
            try {
                return (y[0] * (((value - x[1]) * (value - x[2])) / ((x[0] - x[1]) * (x[0] - x[2])))) +
                        (y[1] * (((value - x[0]) * (value - x[2])) / ((x[1] - x[0]) * (x[1] - x[2])))) +
                        (y[2] * ((  (value - x[0]) * (value - x[1])) / ((x[2] - x[0]) * (x[2] - x[1]))  ));
            } catch (ArithmeticException e) {
                System.out.println(e);
                return -1;
            }
        }
    }

    private static double getBiQuadraticThirdDim(double[][] thirdDimInfo, int indexX, int indexY, boolean useNearest3rdPoint) {

        int x = indexX-1;
        int added = 0;
        List<Integer> indexXList = new ArrayList<>();
        while (added < 2) { //we want first and second left neighbour
            try {
                if (thirdDimInfo[x][indexY] > 0) {
                    indexXList.add(x);
                    added++;
                }
                x--;
            } catch (ArrayIndexOutOfBoundsException e) {
                //we went beyond field, before we got the second neightbour
                //the first neighbour is in worst case the boundary
                break;
            }
        }
        //Collections.sort(indexXList); //todo? we want the smallest index in the first position

        x = indexX+1;
        added = 0;
        while (added < 2) { //we want first and second right neighbour, first two are already in
            try {
                if (thirdDimInfo[x][indexY] > 0) {
                    indexXList.add(x);
                    added++;
                }
                x++;
            } catch (ArrayIndexOutOfBoundsException e) {
               break;
            }

        }
        if (indexXList.size() < 3) {
            System.out.println("impossible indexXList.size() < 3");
            return -1;
        } else if (indexXList.size() == 4) {
            if (indexX - indexXList.get(1) < indexXList.get(3) - indexX) {//todo if list is sorted, set to .get(0) insted of .get(1)
                //left furthest index is closer to point, than right furthest
                if (useNearest3rdPoint){
                    indexXList.remove(3);
                } else {
                    indexXList.remove(1);
                }
            } else {
                if (useNearest3rdPoint){
                    indexXList.remove(1);
                } else {
                    indexXList.remove(3);
                }
            }
        }

        //indeY
        int y = indexY-1;
        added = 0;
        List<Integer> indexYList = new ArrayList<>();
        while (added < 2) {
            try {
                if (thirdDimInfo[indexX][y] > 0) {
                    indexYList.add(y);
                    added++;
                }
                y--;
            } catch (ArrayIndexOutOfBoundsException e) {
                break;
            }
        }
        //Collections.sort(indexXList); //todo? we want the smallest index in the first position

        y = indexY+1;
        added = 0;
        while (added < 2) { //we want first and second right neighbour, first two are already in
            try {
                if (thirdDimInfo[indexX][y] > 0) {
                    indexYList.add(y);
                    added++;
                }
                y++;
            } catch (ArrayIndexOutOfBoundsException e) {
                break;
            }

        }
        if (indexYList.size() < 3) {
            System.out.println("impossible indexYList.size() < 3");
            return -1;
        } else if (indexYList.size() == 4) {
            if (indexY - indexYList.get(1) < indexYList.get(3) - indexY) {//todo if list is sorted, set to .get(0) insted of .get(1)
                //left furthest index is closer to point, than right furthest
                if (useNearest3rdPoint){
                    indexYList.remove(3);
                } else {
                    indexYList.remove(1);
                }
            } else {
                if (useNearest3rdPoint){
                    indexYList.remove(1);
                } else {
                    indexYList.remove(3);
                }
            }
        }

        double valueX = interpolateQuadratic(indexXList.stream().map(val -> (double) val).toArray(Double[]::new), indexXList.stream().map(val -> thirdDimInfo[val][indexY]).toArray(Double[]::new), indexX);
        double valueY = interpolateQuadratic(indexYList.stream().map(val -> (double) val).toArray(Double[]::new), indexYList.stream().map(val -> thirdDimInfo[indexX][val]).toArray(Double[]::new), indexY);

        return (valueX+valueY)/2;
    }

    private static double getBiCubicThirdDim(double[][] thirdDimInfo, int indexX, int indexY) {
        return getBiCubicThirdDim( thirdDimInfo, indexX, indexY, false);
    }

    private static double getBiCubicThirdDim(double[][] thirdDimInfo, int indexX, int indexY, boolean spline) {

        int x = indexX-1;
        int added = 0;
        List<Integer> indexXList = new ArrayList<>();
        while (added < 2) { //we want first and second left neighbour
            try {
                if (thirdDimInfo[x][indexY] > 0) {
                    indexXList.add(x);
                    added++;
                }
                x--;
            } catch (ArrayIndexOutOfBoundsException e) {
                //we went beyond field, before we got the second neightbour
                //set the second neighboour
                indexXList.add(-1);
                break;
            }
        }

        x = indexX+1;
        added = 0;
        while (added < 2) { //we want first and second right neighbour, first two are already in
            try {
                if (thirdDimInfo[x][indexY] > 0) {
                    indexXList.add(x);
                    added++;
                }
                x++;
            } catch (ArrayIndexOutOfBoundsException e) {
                indexXList.add(thirdDimInfo.length);
                break;
            }

        }

        //indeY
        int y = indexY-1;
        added = 0;
        List<Integer> indexYList = new ArrayList<>();
        while (added < 2) {
            try {
                if (thirdDimInfo[indexX][y] > 0) {
                    indexYList.add(y);
                    added++;
                }
                y--;
            } catch (ArrayIndexOutOfBoundsException e) {
                indexYList.add(-1);
                break;
            }
        }

        y = indexY+1;
        added = 0;
        while (added < 2) { //we want first and second right neighbour, first two are already in
            try {
                if (thirdDimInfo[indexX][y] > 0) {
                    indexYList.add(y);
                    added++;
                }
                y++;
            } catch (ArrayIndexOutOfBoundsException e) {
                indexYList.add(thirdDimInfo[indexX].length);
                break;
            }
        }

        Collections.sort(indexXList);
        Collections.sort(indexYList);

        double valueX = CubicInterpolate(getDoubleArrayFromIntList(indexXList), indexXList.stream().map(val -> {
            if (val == -1) {
                return 2* (thirdDimInfo[indexXList.get(1)][indexY]) - thirdDimInfo[indexXList.get(2)][indexY];//exterpolate, 2*p1 - p2
            } else if (val == thirdDimInfo.length) {
                return 2 * (thirdDimInfo[indexXList.get(2)][indexY]) - thirdDimInfo[indexXList.get(1)][indexY];
            } else {
                return thirdDimInfo[val][indexY]; //index not out of bounds
            }
        }).toArray(Double[]::new), indexX, spline);
        double valueY = CubicInterpolate(getDoubleArrayFromIntList(indexYList), indexYList.stream().map(val -> {
            if (val == -1) {
                return 2* (thirdDimInfo[indexX][indexYList.get(1)]) - thirdDimInfo[indexX][indexYList.get(2)];
            } else if (val == thirdDimInfo[indexX].length) {
                return 2 * (thirdDimInfo[indexX][indexYList.get(2)]) - thirdDimInfo[indexX][indexYList.get(1)];
            } else {
                return thirdDimInfo[indexX][val]; //index not out of bounds
            }
        }).toArray(Double[]::new), indexY, spline);

//        double valueX = cubicInterpolation(getDoubleArrayFromIntList(indexXList), indexX);
//        double valueY = cubicInterpolation(getDoubleArrayFromIntList(indexYList), indexY);
//        return valueY;
        return (valueX+valueY)/2;
//        return -1;


//        double[][] fvalue = new double[4][4];
//        for (int i = 0; i<fvalue.length; i++) {
//            for (int j = 0; j<fvalue[i].length; j++) {
//                int xx = indexXList.get(i);
//                int yy = indexYList.get(i);
//
//                try {
//                    fvalue[i][j] = thirdDimInfo[xx][yy];
//                } catch (ArrayIndexOutOfBoundsException e) {
//                    if (xx == -1) {
//                        fvalue[i][j] = thirdDimInfo[0][yy];
//                    } else if (xx == thirdDimInfo.length) {
//                        fvalue[i][j] = thirdDimInfo[thirdDimInfo.length-1][yy];
//                    } else if (yy == -1) {
//                        fvalue[i][j] = thirdDimInfo[xx][0];
//                    } else if (yy == thirdDimInfo[xx].length) {
//                        fvalue[i][j] = thirdDimInfo[xx][thirdDimInfo[xx].length-1];
//                    } else {
//                        System.out.println("napaka: " + e);
//                        return -1;
//                    }
//                }
//            }
//        }
//
//        BicubicInterpolator bi = new BicubicInterpolator();
//        BicubicInterpolatingFunction function = bi.interpolate(getDoubleArrayFromIntList(indexXList), getDoubleArrayFromIntList(indexYList), fvalue);
//        return function.value(indexX, indexY);
    }

    public static double cubicInterpolation(double[] p, double value) {
        double normalizedValue = (value-p[1])/(p[2]-p[1]); // https://stats.stackexchange.com/questions/70801/how-to-normalize-data-to-0-1-range
        //https://www.paulinternet.nl/?page=bicubic
        return p[1] + 0.5 * normalizedValue*(p[2] - p[0] + normalizedValue*(2.0*p[0] - 5.0*p[1] + 4.0*p[2] - p[3] + normalizedValue*(3.0*(p[1] - p[2]) + p[3] - p[0])));
    }

    public static double CubicInterpolate(
            double[] points,
            Double[] values,
            double mu
            ) {

        return CubicInterpolate(points, values, mu, false);
    }

     public static double CubicInterpolate(
            double[] points,
            Double[] values,
            double mu,
            boolean catmullRom)
    {
        double originalValue = mu; //temp todo
        mu = (mu-points[1])/(points[2]-points[1]); //normalize
        double y0 = values[0];
        double y1 = values[1];
        double y2 = values[2];
        double y3 = values[3];
        double a0,a1,a2,a3,mu2;

        //http://paulbourke.net/miscellaneous/interpolation/
        mu2 = mu*mu;

        if (catmullRom) {
            a0 = -0.5*y0 + 1.5*y1 - 1.5*y2 + 0.5*y3;
            a1 = y0 - 2.5*y1 + 2*y2 - 0.5*y3;
            a2 = -0.5*y0 + 0.5*y2;
            a3 = y1;


            //todo uredi, naredi več različnih interpolacij, več splinpv, ...
            return splineInterpolation2(points, values,originalValue);

        } else {
            a0 = y3 - y2 - y0 + y1;
            a1 = y0 - y1 - a0;
            a2 = y2 - y0;
            a3 = y1;
        }

        double a = a0*mu*mu2+a1*mu2+a2*mu+a3;
        if (a > 411000) {
            double c = a;
        }
        return(a0*mu*mu2+a1*mu2+a2*mu+a3);
    }

    public static double catmullRom( double[] points,
                        Double[] values,
                        double mu) {
        return CubicInterpolate(points, values, mu, true);
    }

    private static double[] getDoubleArrayFromIntList(List<Integer> list) {
        return ArrayUtils.toPrimitive(list.stream().map(x -> (double) x).toArray(Double[]::new));
    }

    public static double splineInterpolation(double[] points, double t){


        t = (t-points[1])/(points[2]-points[1]); //normalize
        double p0 = points[0];
        double p1 = points[1];
        double p2 = points[2];
        double p3 = points[3];

        double  d1,d2;
        d1=0.5*(p2-p0);
        d2=0.5*(p3-p1);
        double a0=p1;
        double a1=d1;
        double a2=(3.0*(p2-p1))-(2.0*d1)-d2;
        double a3=d1+d2+(2.0*(-p2+p1));

        return a0+a1*t+a2*t*t+a3*t*t*t;

    }

    private static double splineInterpolation2(double[] points, Double[] values, double t){
        SplineInterpolator si = new SplineInterpolator();
        return si.interpolate(points,ArrayUtils.toPrimitive(values)).value(t);
    }

    private static double getSplineThirdDim(double[][] thirdDimInfo, int indexX, int indexY) {

        return getBiCubicThirdDim(thirdDimInfo,indexX,indexY,true);
    }
}
