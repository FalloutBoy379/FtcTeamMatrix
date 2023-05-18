package org.firstinspires.ftc.teamcode.util;

public class MatrixUtils {
    public static double[][] multiplyMatrices(double[][] matrix1, double[][] matrix2) {
        int m1Rows = matrix1.length;
        int m1Cols = matrix1[0].length;
        int m2Cols = matrix2[0].length;

        double[][] result = new double[m1Rows][m2Cols];

        for (int i = 0; i < m1Rows; i++) {
            for (int j = 0; j < m2Cols; j++) {
                double sum = 0;
                for (int k = 0; k < m1Cols; k++) {
                    sum += matrix1[i][k] * matrix2[k][j];
                }
                result[i][j] = sum;
            }
        }

        return result;
    }

    public static double[][] addMatrices(double[][] matrix1, double[][] matrix2) {
        int rows = matrix1.length;
        int cols = matrix1[0].length;

        double[][] result = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = matrix1[i][j] + matrix2[i][j];
            }
        }

        return result;
    }

    public static double[][] invertMatrix(double[][] matrix) {
        double det = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
        double invDet = 1 / det;

        double[][] result = new double[2][2];
        result[0][0] = matrix[1][1] * invDet;
        result[0][1] = -matrix[0][1] * invDet;
        result[1][0] = -matrix[1][0] * invDet;
        result[1][1] = matrix[0][0] * invDet;

        return result;
    }

    public static double[][] transposeMatrix(double[][] matrix) {
        int rows = matrix.length;
        int cols = matrix[0].length;

        double[][] result = new double[cols][rows];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[j][i] = matrix[i][j];
            }
        }

        return result;
    }

    public static double[][] multiplyScalar(double scalar, double[][] matrix) {
        int rows = matrix.length;
        int cols = matrix[0].length;

        double[][] result = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = scalar * matrix[i][j];
            }
        }

        return result;
    }
}
