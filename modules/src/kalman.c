/*
 * kalman.c
 *
 *  Created on: Mar 19, 2015
 *      Author: jianxin
 */

//kalman run at 100 Hz, dt = 1/100
static float A[2][2] = { { 1, 0.01 }, { 0, 1 } };
static float transA[2][2] = { { 1, 0 }, { 0.01, 1 } };
static float B[2] = { 0, 0.01 };
static float H[2][2] = { { 1, 0 }, { 0, 0 } };
static float transH[2][2] = { { 1, 0 }, { 0, 0 } };
static float Q[2][2] = { { 1, 0 }, { 0, 10 } };
static float R[2][2] = { { 100, 0 }, { 0, 1 } };

static void matrix2Product(float a[2][2], float b[2][2], float c[2][2]);
static void matrix2Plus(float a[2][2], float b[2][2], float c[2][2]);
static void matrix2Inverse(float a[2][2], float inva[2][2]);

void kalman_update(float xhat_k_1[2], float z_k[2], float u_k_1,
		float P_k_1[2][2], float xhat_k[2], float P_k[2][2], float dt) {
	float xhat_k_[2] = { 0, 0 };
	xhat_k_[0] = A[0][0] * xhat_k_1[0] + A[0][1] * xhat_k_1[1] + u_k_1 * B[0];
	xhat_k_[1] = A[1][0] * xhat_k_1[0] + A[1][1] * xhat_k_1[1] + u_k_1 * B[1];

	float P_k_[2][2] = { { 0, 0 }, { 0, 0 } };
	float m1[2][2] = { { 0, 0 }, { 0, 0 } };
	float m2[2][2] = { { 0, 0 }, { 0, 0 } };
	float m3[2][2] = { { 0, 0 }, { 0, 0 } };

	matrix2Product(A, P_k_1, m1);
	matrix2Product(m1, transA, m2);
	matrix2Plus(m2, Q, P_k_);

	float K_k[2][2] = { { 0, 0 }, { 0, 0 } };
	matrix2Product(P_k_, transH, m3);
	matrix2Product(H, m3, m1);
	matrix2Plus(m1, R, m2);
	matrix2Inverse(m2, m1);
	matrix2Product(m3, m1, K_k);

	float ek[2] = { 0, 0 };
	ek[0] = z_k[0] - H[0][0] * xhat_k_[0] - H[0][1] * xhat_k_[1];
	ek[1] = z_k[1] - H[1][0] * xhat_k_[0] - H[1][1] * xhat_k_[1];

	xhat_k[0] = xhat_k_[0] + K_k[0][0] * ek[0] + K_k[0][1] * ek[1];
	xhat_k[1] = xhat_k_[1] + K_k[1][0] * ek[0] + K_k[1][1] * ek[1];

	matrix2Product(K_k, H, m1);
	m2[0][0] = 1 - m1[0][0];
	m2[0][1] = -m1[0][1];
	m2[1][0] = -m1[1][0];
	m2[1][1] = 1 - m1[1][1];
	matrix2Product(m2, P_k_, P_k);

}

static void matrix2Product(float a[2][2], float b[2][2], float c[2][2]) {
	int i, j, k;
	for (i = 0; i < 2; i++)
		for (j = 0; j < 2; j++) {
			c[i][j] = 0;
			for (k = 0; k < 2; k++)
				c[i][j] += a[i][k] * b[k][j];
		}
}

static void matrix2Plus(float a[2][2], float b[2][2], float c[2][2]) {
	int i, j;
	for (i = 0; i < 2; i++)
		for (j = 0; j < 2; j++)
			c[i][j] = a[i][j] + b[i][j];
}

static void matrix2Inverse(float a[2][2], float inva[2][2]) {
	float inv_norm = 1 / (a[0][0] * a[1][1] - a[0][1] * a[1][0]);

	inva[0][0] = inv_norm * a[1][1];
	inva[0][1] = -inv_norm * a[0][1];
	inva[1][0] = -inv_norm * a[1][0];
	inva[1][1] = inv_norm * a[0][0];
}
