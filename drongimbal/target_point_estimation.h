#pragma once





#include <stdint.h>
#include <stdbool.h>




typedef struct Vector3
{
    float f32Val[3][1];
} tVector3_1;
typedef struct matrix3_3
{
    float f32Val[3][3];
}tMatrix3_3;

int32_t vector_addition3_1(tVector3_1* tInput1, tVector3_1* tInput2, tVector3_1* tVector_add);
int32_t vector_subtraction3_1(tVector3_1* tInput1, tVector3_1* tInput2, tVector3_1* tVector_sub);
int32_t vector_scalar_multiply3_1(tVector3_1* tInput1, float f32scalar, tVector3_1* tVector_scalar_multi);
int32_t matrix_vector_multiply3_1(tMatrix3_3* InputMat, tVector3_1* Inputvec, tVector3_1* tmat_vec_multi);
int32_t matrix_matrix_multiply3_3(tMatrix3_3* FrontMat, tMatrix3_3* BehindMat, tMatrix3_3* OutMat);
float vector_Euclid_norm3_1(tVector3_1* Inputvec);
void Yaw_Z_Rotation_Mat(float YawAngle_rad, tMatrix3_3* z_Rotation_Mat);
void Pitch_Y_Rotation_Mat(float PitchAngle_rad, tMatrix3_3* y_Rotation_Mat);
void Roll_X_Rotation_Mat(float RollAngle_rad, tMatrix3_3* x_Rotation_Mat);
void Inv_Rotation_Mat(tMatrix3_3* Rotation_Mat, tMatrix3_3* inv_Rot_Mat);
void find_Gimbal_Direction();