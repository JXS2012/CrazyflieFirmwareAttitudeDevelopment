/*
 * kalman.c
 *
 *  Created on: Mar 19, 2015
 *      Author: jianxin
 */
#include "better_matrix.h"
//kalman run at 100 Hz, dt = 1/100
static float valueA[4] = {  1, 0.01 ,  0, 1  };
static float valueB[2] = { 0, 0.01 };
static float valueH[4] = {  1, 0,  0, 0  };
static float valueQ[4] = {  1, 0 ,  0, 10  };
static float valueR[4] = {  100, 0 ,  0, 1  };
static float valueI[4] = {1,0,0,1};

void kalman_updateMatrix(Matrix xhat_k_1, Matrix z_k,float u_k_1, Matrix P_k_1,
		Matrix *xhat_k, Matrix *P_k) {

	Matrix A = initMatrixValue(2,2,valueA);
	Matrix transA = transposeMatrixBetter(A);
	Matrix B = initMatrixValue(2,1,valueB);
	Matrix H = initMatrixValue(2,2,valueH);
	Matrix transH = transposeMatrixBetter(H);
	Matrix Q = initMatrixValue(2,2,valueQ);
	Matrix R = initMatrixValue(2,2,valueR);
	Matrix I = initMatrixValue(2,2,valueI);

	Matrix xhat_k_ = matrixPlus(matrixProductBetter(A, xhat_k_1),
			scalarMatrixProductBetter(u_k_1, B));

	Matrix P_k_ = matrixPlus(Q, matrixProductBetter(A, matrixProductBetter(P_k_1, transA)));

	Matrix K_k = matrixProductBetter(matrixProductBetter(P_k_, transH),
			inverseMatrix3(
					matrixPlus(R,
							matrixProductBetter(H,
									matrixProductBetter(P_k_, transH)))));

	Matrix ek = matrixSubtraction(z_k, matrixProductBetter(H, xhat_k_));

	*xhat_k = matrixPlus(xhat_k_, matrixProductBetter(K_k, ek));

	*P_k = matrixProductBetter(matrixSubtraction(I, matrixProductBetter(K_k, H)), P_k_);

}

