#ifndef MATRIX_H_
#define MATRIX_H_

float normMatrix(float input[3][3]);
void subMatrix(float input[3][3], float output[3][3]);
void transposeMatrix(float input[3][3], float output[3][3]);
void scalarMatrixProduct(float scale, float input[3][3], float output[3][3]);
void inverseMatrix(float input[3][3], float output[3][3]);

void matrixVectorProduct(float A[3][3], float b[3], float c[3]);
void matrixMinus(float a[3][3], float b[3][3], float c[3][3]);
void matrixProduct(float A[3][3], float B[3][3], float C[3][3]);

void scalarVectorProduct(float k, float in[3], float out[3]);
void vectorPlus(float a[3], float b[3], float c[3]);
void vectorMinus(float a[3], float b[3], float c[3]);
void vectorMatrixProduct(float a[3], float B[3][3], float c[3]);

void normalizeVector(float a[3], float b[3]);
float invSqrt(float x);
float vectorNorm2(float a[3]);

#endif /* MATRIX_H_ */
