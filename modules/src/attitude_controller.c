/*
 * attitude_controller.c
 *
 *  Created on: Mar 13, 2015
 *      Author: jianxin
 */
#include "matrix.h"

void gradientVarphi(float R[3][3] , float grad[3] );
void pseudo_inver_skew(float A[3][3], float a[3]);
void inver_skew(float A[3][3], float a[3]);
void skew(float a[3], float ax[3][3]);

void attitudeControl(float R[3][3], float D[3][3], float d[3], float ddot[3], float w[3], float u[3])
{
    //float M[3][3] = {{1,0,0},{0,1,0},{0,0,1}};

    float m1[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    float m2[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

    float Jw[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    float transE[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    float transEd[3] = {0,0,0};

    float v1[3] = {0,0,0};
    float v2[3] = {0,0,0};
    //float v3[3] = {0,0,0};

    float u1[3] = {0,0,0};
    float u2[3] = {0,0,0};

    float k1 = -1;
    float k2 = -1;
    transposeMatrix(D, m1);
    matrixProduct(m1, R, m2);
    transposeMatrix(m2, transE);

    matrixVectorProduct(transE, d, transEd);
    vectorMinus(w, transEd, v1);
    scalarVectorProduct(k2, v1, u);

    gradientVarphi(m2, v2);
    scalarVectorProduct(k1, v2, u2);

    vectorPlus(u, u2, u1);
/*
    u[0] = u1[0];
    u[1] = u1[1];
    u[2] = u1[2];
*/
    float k3 = 0.01;
    skew(transEd, m1);
    matrixVectorProduct(m1, v1, u2);
    scalarVectorProduct(k3, u2, u);
    vectorPlus(u1, u, u2);

    matrixVectorProduct(transE, ddot, v1);
    skew(w,Jw);
    matrixVectorProduct(Jw, transEd, v2);
    vectorMinus(v1, v2, u1);
    scalarVectorProduct(k3, u1, u);
    vectorPlus(u, u2, u1);

    matrixProduct(Jw, transE, m1);
    matrixVectorProduct(m1, d, u);
    scalarVectorProduct(k3, u, u2);
    vectorPlus(u1, u2, u);
/*
    skew(transEd, m1);
    matrixProduct(m1, M, m2);
    matrixVectorProduct(m2, v1, u2);

    vectorPlus(u1, u2, u);

    matrixVectorProduct(transE, ddot, v1);
    skew(w,Jw);
    matrixVectorProduct(Jw, transEd, v2);
    vectorMinus(v1, v2, v3);
    matrixVectorProduct(M, v3, u1);
    vectorPlus(u, u1, u2);

    matrixProduct(Jw, M, m1);
    matrixProduct(m1, transE, m2);
    matrixVectorProduct(m2, d, u1);
    vectorPlus(u2, u1, u);*/
}

void gradientVarphi(float R[3][3] , float grad[3] )
{
	float invM[3][3] = {{100,0,0},{0,100,0},{0,0,100}};
	float pseudo_inver_skew_PR[3] = {0,0,0};

	float P[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}};
	float PR[3][3] = {{0,0,0}, {0,0,0}, {0,0,0}};

	matrixProduct(P, R, PR);

	pseudo_inver_skew(PR, pseudo_inver_skew_PR);

	matrixVectorProduct(invM, pseudo_inver_skew_PR, grad);
}

void pseudo_inver_skew(float A[3][3], float a[3])
{
	float transA[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
	float residueA[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
	transposeMatrix(A, transA);
	matrixMinus(A, transA, residueA);
	inver_skew(residueA, a);
}

void inver_skew(float A[3][3], float a[3])
{
	a[0] = A[2][1];
	a[1] = A[0][2];
	a[2] = A[1][0];
}

void skew(float a[3], float ax[3][3])
{
	ax[0][0] = 0;
	ax[0][1] = -a[2];
	ax[0][2] = a[1];
	ax[1][0] = a[2];
	ax[1][1] = 0;
	ax[1][2] = -a[0];
	ax[2][0] = -a[1];
	ax[2][1] = a[0];
	ax[2][2] = 0;
}
