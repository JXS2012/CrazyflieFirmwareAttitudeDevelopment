/*
 * matrix.c
 *
 *  Created on: Mar 27, 2015
 *      Author: jianxin
 */

#include "better_matrix.h"
//#include "stdlib.h"

Matrix subMatrix3(Matrix a);

Matrix matrixSubtraction(Matrix a, Matrix b) {
	Matrix c = initMatrix(a.row, a.column);
	int i, j;
	for (i = 0; i < a.row; i++)
		for (j = 0; j < a.column; j++)
			*(c.head + i * c.column + j) = *(a.head + i * a.column + j)
					- *(b.head + i * b.column + j);
	return c;
}

Matrix matrixPlus(Matrix a, Matrix b) {
	Matrix c = initMatrix(a.row, a.column);
	int i, j;
	for (i = 0; i < a.row; i++)
		for (j = 0; j < a.column; j++)
			*(c.head + i * c.column + j) = *(a.head + i * a.column + j)
					+ *(b.head + i * b.column + j);
	return c;
}

float detMatrix3(Matrix a) {
	if ((a.row == 3) && (a.column == 3)) {
		float sum1 = (*a.head) * (*(a.head + 4)) * (*(a.head + 8))
				+ (*a.head + 1) * (*(a.head + 5)) * (*(a.head + 6))
				+ (*a.head + 2) * (*(a.head + 3)) * (*(a.head + 7));
		float sum2 = (*a.head) * (*(a.head + 5)) * (*(a.head + 7))
				+ (*a.head + 1) * (*(a.head + 3)) * (*(a.head + 8))
				+ (*a.head + 2) * (*(a.head + 4)) * (*(a.head + 6));
		return sum1 - sum2;
	} else if ((a.row == 2) && (a.column == 2)) {
		return (*a.head) * (*(a.head + 3)) - (*(a.head + 1)) * (*(a.head + 2));
	} else if ((a.row == 1) && (a.column == 1)) {
		return *a.head;
	} else
		return 0;
}

Matrix subMatrix3(Matrix a) {
	Matrix t = initMatrix(a.row, a.column);

	int i, j;

	for (i = 0; i < t.row; i++)
		for (j = 0; j < t.column; j++) {
			Matrix subMatrix = initMatrix(a.row - 1, a.column - 1);
			int m, n, k;
			k = 0;
			for (m = 0; m < t.row; m++)
				for (n = 0; n < t.row; n++)
					if (m != i && n != j) {
						*(subMatrix.head + k) = *(a.head + m * a.column + n);
						k += 1;
					}
			if (t.column == 3 && t.row == 3)
				if ((i * t.column + j) % 2 == 0)
					*(t.head + i * t.column + j) = detMatrix3(subMatrix);
				else
					*(t.head + i * t.column + j) = (-1) * detMatrix3(subMatrix);
			else if (t.column == 2 && t.row == 2)
				switch (i * t.column + j) {
				case 0:
					*(t.head + i * t.column + j) = detMatrix3(subMatrix);
					break;
				case 1:
					*(t.head + i * t.column + j) = -detMatrix3(subMatrix);
					break;
				case 2:
					*(t.head + i * t.column + j) = -detMatrix3(subMatrix);
					break;
				case 3:
					*(t.head + i * t.column + j) = detMatrix3(subMatrix);
					break;
				}
		}

	return t;
}

Matrix transposeMatrixBetter(Matrix a) {
	Matrix t = initMatrix(a.column, a.row);
	int i, j;
	for (i = 0; i < t.row; i++)
		for (j = 0; j < t.column; j++)
			*(t.head + i * t.column + j) = *(a.head + j * a.column + i);
	return t;
}

Matrix inverseMatrix3(Matrix a) {
	Matrix t;
	t = subMatrix3(a);
	t = transposeMatrixBetter(t);
	t = scalarMatrixProductBetter(1 / detMatrix3(a), t);
	return t;
}

float norm2(Matrix a) {
	int i, j;
	float norm = 0;
	for (i = 0; i < a.row; i++)
		for (j = 0; j < a.column; j++)
			norm += *(a.head + i * a.column + j)
					* (*(a.head + i * a.column + j));
	return norm;
}

Matrix matrixProductBetter(Matrix a, Matrix b) {
	Matrix c = initMatrix(a.row, b.column);
	int i, j, k;
	for (i = 0; i < c.row; i++)
		for (j = 0; j < c.column; j++) {
			*(c.head + i * c.column + j) = 0;
			for (k = 0; k < a.column; k++)
				*(c.head + i * c.column + j) += *(a.head + i * a.column + k)
						* (*(b.head + k * b.column + j));
		}
	return c;
}

Matrix scalarMatrixProductBetter(float k, Matrix a) {
	Matrix t = initMatrix(a.row, a.column);
	int i, j;
	for (i = 0; i < t.row; i++)
		for (j = 0; j < t.row; j++)
			*(t.head + i * t.column + j) = *(a.head + i * t.column + j) * k;
	return t;
}

Matrix initMatrix(int row, int column) {
	Matrix t;
	t.row = row;
	t.column = column;
	int i;
	for (i = 0; i < row*column; i++)
		t.head[i] = 0;
	return t;
}

Matrix initMatrixValue(int row, int column, float *array) {
	Matrix t = initMatrix(row, column);
	t.row = row;
	t.column = column;
	int i;
	for (i = 0; i < row*column; i ++)
		t.head[i] = array[i];
	return t;
}

Matrix normalizeMatrix(Matrix a) {
	return scalarMatrixProductBetter(invSqrtBetter(norm2(a)), a);
}

float invSqrtBetter(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*) &y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*) &i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
