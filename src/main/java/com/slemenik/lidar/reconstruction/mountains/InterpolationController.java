package com.slemenik.lidar.reconstruction.mountains;

import java.util.ArrayList;
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
        AVERAGE_24N
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
            default:
                System.out.println("wrong interpolation param");
                return -1;
        }
    }

    private double getAverage16NThirdDim(double[][] thirdDimInfo, int indexX, int indexY) {
        return 0;
    }

}
