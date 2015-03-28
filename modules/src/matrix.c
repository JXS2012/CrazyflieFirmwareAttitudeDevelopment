#include <math.h>
float normMatrix(float input[3][3])
{
  float sum1 = input[0][0]*input[1][1]*input[2][2] + input[0][1]*input[1][2]*input[2][0] + input[0][2]*input[1][0]*input[2][1];
  float sum2 = input[0][0]*input[1][2]*input[2][1] + input[0][1]*input[1][0]*input[2][2] + input[0][2]*input[1][1]*input[2][0];
  return sum1 - sum2;
}

void subMatrix(float input[3][3], float output[3][3])
{
  output[0][0] = input[1][1]*input[2][2]-input[1][2]*input[2][1];
  output[0][1] = -(input[1][0]*input[2][2]-input[1][2]*input[2][0]);
  output[0][2] = input[1][0]*input[2][1]-input[1][1]*input[2][0];

  output[1][0] = -(input[0][1]*input[2][2]-input[0][2]*input[2][1]);
  output[1][1] = input[0][0]*input[2][2]-input[0][2]*input[2][0];
  output[1][2] = -(input[0][0]*input[2][1]-input[0][1]*input[2][0]);

  output[2][0] = input[0][1]*input[1][2]-input[1][1]*input[0][2];
  output[2][1] = -(input[0][0]*input[1][2]-input[0][2]*input[1][0]);
  output[2][2] = input[1][1]*input[0][0]-input[1][0]*input[0][1];
}

void transposeMatrix(float input[3][3], float output[3][3])
{
	int i,j;
	for (i = 0; i<3; i++)
		for (j = 0; j<3; j++)
			output[i][j] = input[j][i];
}

void scalarMatrixProduct(float scale, float input[3][3], float output[3][3])
{
  int i,j;
  for (i = 0; i<3; i++)
    for (j = 0; j<3; j++)
      output[i][j] = scale * input[i][j];
}

void inverseMatrix(float input[3][3], float output[3][3])
{
  subMatrix(input, output);
  float transOutput[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
  transposeMatrix(output, transOutput);
  scalarMatrixProduct(normMatrix(input),transOutput, output);
}

void matrixProduct(float A[3][3], float B[3][3], float C[3][3])
{
	int i,j,k;
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
		{
			C[i][j] = 0;
			for (k = 0; k < 3; k++)
			C[i][j] += A[i][k]*B[k][j];
		}
}

void scalarVectorProduct(float k, float in[3], float out[3])
{
	int i;
	for (i = 0; i < 3; i ++)
		out[i] = k * in[i];
}

void vectorPlus(float a[3], float b[3], float c[3])
{
	int i;
	for (i = 0; i < 3; i ++)
		c[i] = a[i] + b[i];
}

void vectorMinus(float a[3], float b[3], float c[3])
{
	int i;
	for (i = 0; i < 3; i ++)
		c[i] = a[i] - b[i];
}

void matrixVectorProduct(float A[3][3], float b[3], float c[3])
{
	int i,j;
	for (i = 0; i < 3; i ++)
	{
		c[i] = 0;
		for (j = 0; j < 3; j ++)
			c[i] += A[i][j]*b[j];
	}
}

void matrixMinus(float a[3][3], float b[3][3], float c[3][3])
{
	int i,j;
	for (i = 0; i < 3; i ++)
		for (j = 0; j < 3; j ++)
			c[i][j] = a[i][j] - b[i][j];
}

void vectorMatrixProduct(float a[3], float B[3][3], float c[3])
{
	int i,j;
	for (i = 0; i < 3; i ++)
	{
		c[i] = 0;
		for (j = 0; j < 3; j ++)
			c[i] += a[j] * B[j][i];
	}
}

float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

void normalizeVector(float a[3], float b[3])
{
	float invnorm;
	invnorm = invSqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
	int i = 0;
	for (i = 0; i < 3; i ++)
	{
		b[i] = a[i]*invnorm;
	}
}

float vectorNorm2(float a[3])
{
	return a[0]*a[0] + a[1]*a[1] + a[2]*a[2];
}
