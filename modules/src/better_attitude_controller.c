/*
 * attitude_controller.c
 *
 *  Created on: Mar 13, 2015
 *      Author: jianxin
 */
#include "better_matrix.h"
#include "better_attitude_controller.h"

Matrix gradientVarphiMatrix(Matrix R);
Matrix pseudo_inver_skewMatrix(Matrix A);
Matrix inver_skewMatrix(Matrix A);
Matrix skewMatrix(Matrix a);

Matrix attitudeControlMatrix(Matrix R, Matrix D, Matrix d, Matrix ddot, Matrix w)
{
    Matrix transD = transposeMatrixBetter(D);
    Matrix E = matrixProductBetter(transD, R);
    Matrix transE = transposeMatrixBetter(E);
    Matrix transEd = matrixProductBetter(transE, d);
    Matrix e = matrixSubtraction(w, transEd);

    float k2 = -1;
    Matrix u = scalarMatrixProductBetter(k2, e);

    float k1 = -1;
    u = matrixPlus(u, scalarMatrixProductBetter(k1, gradientVarphiMatrix(E)));

	int i,j;
	Matrix M = initMatrix(3,3);
	for (i = 0; i < 3; i ++)
		for (j = 0; j < 3; j ++)
			if (i == j)
				*(M.head + i*3 + j) = 0.01;
			else
				*(M.head + i*3 + j) = 0;

    u = matrixPlus(u,matrixProductBetter(matrixProductBetter(skewMatrix(transEd), M), e));

    Matrix Jw = skewMatrix(w);
    u = matrixPlus(u,matrixProductBetter(M, matrixSubtraction( matrixProductBetter(transE, ddot), matrixProductBetter(Jw, transEd))));

    u = matrixPlus(u, matrixProductBetter(matrixProductBetter(Jw, M), transEd));

    return u;
}

Matrix gradientVarphiMatrix(Matrix R)
{
	int i,j;
	Matrix invM = initMatrix(3,3);
	for (i = 0; i < 3; i ++)
		for (j = 0; j < 3; j ++)
			if (i == j)
				*(invM.head + i*3 + j) = 100;
			else
				*(invM.head + i*3 + j) = 0;

	Matrix P = initMatrix(3,3);
	for (i = 0; i < 3; i ++)
		for (j = 0; j < 3; j ++)
			if (i == j)
				*(invM.head + i*3 + j) = 1;
			else
				*(invM.head + i*3 + j) = 0;

	return matrixProductBetter(invM, pseudo_inver_skewMatrix(matrixProductBetter(P, R)));
}

Matrix pseudo_inver_skewMatrix(Matrix A)
{
	return inver_skewMatrix(matrixSubtraction(A, transposeMatrixBetter(A)));
}

Matrix inver_skewMatrix(Matrix A)
{
	Matrix a = initMatrix(3,1);
	*(a.head) = *(A.head + 2*A.column + 1);
	*(a.head+1) = *(A.head + 0*A.column + 2);
	*(a.head+2) = *(A.head + 1*A.column + 0);
	return a;
}

Matrix skewMatrix(Matrix a)
{
	Matrix ax = initMatrix(3,3);
	*(ax.head) = 0;
	*(ax.head+1) = -*(a.head + 2);
	*(ax.head+2) = *(a.head + 1);
	*(ax.head+3) = *(a.head + 2);
	*(ax.head+4) = 0;
	*(ax.head+5) = -*(a.head);
	*(ax.head+6) = -*(a.head + 1);
	*(ax.head+7) = *(a.head);
	*(ax.head+8) = 0;
	return ax;
}
