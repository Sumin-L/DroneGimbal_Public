#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "target_point_estimation.h"



tMatrix3_3 tDroneRoll_Mat, tDronePitch_Mat, tDroneYaw_Mat;
float f32DroneRoll_rad, f32DronePitch_rad, f32DroneYaw_rad;

tMatrix3_3 tGimbalPitch_Mat, tGimbalYaw_Mat;
float f32GimbalPitch_rad, f32GimbalYaw_rad;
float f32Ratio;
tVector3_1 tGimbalDirection_Vec;
float f32GimbalDirection_Vec_Norm;

float f32DroneXposition_m, f32DroneYposition_m, f32DroneAltitude_m;
float f32GimbalDirectionYaw_rad, f32GimbalDirectionAngle_rad;
float f32TargetXYlength_m;
float f32TargetX_m, f32TargetY_m;
float f32TargetX1_m, f32TargetY1_m;
int main() {
    f32DroneYaw_rad = 10.0 * 3.141592 / 180.0;
    f32DronePitch_rad = 10.0 * 3.141592 / 180.0;
    f32DroneRoll_rad = 15.0 * 3.141592 / 180.0;
    f32GimbalYaw_rad = 20.0 * 3.141592 / 180.0;
    f32GimbalPitch_rad = -30.0 * 3.141592 / 180.0;
    f32DroneXposition_m = 0.0;
    f32DroneYposition_m = 0.0;
    f32DroneAltitude_m = 80;

  

    find_Gimbal_Direction();
    f32TargetX1_m = tGimbalDirection_Vec.f32Val[0][0] * f32Ratio + f32DroneXposition_m;
    f32TargetY1_m = tGimbalDirection_Vec.f32Val[1][0] * f32Ratio + f32DroneXposition_m;
    f32TargetXYlength_m = tanf(f32GimbalDirectionAngle_rad) * f32DroneAltitude_m;
    f32TargetX_m = f32TargetXYlength_m * cosf(f32GimbalDirectionYaw_rad) + f32DroneXposition_m;
    f32TargetY_m = f32TargetXYlength_m * sinf(f32GimbalDirectionYaw_rad) + f32DroneYposition_m;
}
    /*void main_target_point_estimation() {
        f32DroneYaw_rad = 10.0 * 3.141592 / 180.0;
        f32DronePitch_rad = 10.0 * 3.141592 / 180.0;
        f32DroneRoll_rad = 15.0 * 3.141592 / 180.0;
        f32GimbalYaw_rad = 20.0 * 3.141592 / 180.0;
        f32GimbalPitch_rad = -0.5249;
        f32DroneXposition_m = 0.0;
        f32DroneYposition_m = 0.0;
        f32DroneAltitude_m = 80;




        find_Gimbal_Direction();
        f32TargetXYlength_m = tanf(f32GimbalDirectionAngle_rad) * f32DroneAltitude_m;
        f32TargetX_m = f32TargetXYlength_m * cosf(f32GimbalDirectionYaw_rad) + f32DroneXposition_m;
        f32TargetY_m = f32TargetXYlength_m * sinf(f32GimbalDirectionYaw_rad) + f32DroneYposition_m;

    }*/

void find_Gimbal_Direction() {
    tVector3_1 X_basis_vec; /// Gimbal coordinate X basis vector
    X_basis_vec.f32Val[0][0] = 1.0;
    X_basis_vec.f32Val[1][0] = 0.0;
    X_basis_vec.f32Val[2][0] = 0.0;
    tVector3_1 Cal_1_Vec, Cal_2_Vec, Cal_3_Vec, Cal_4_Vec; /// result of calculation
    float Gimbal_X_direc, Gimbal_Y_direc, Gimbal_Z_direc; /// sx, sy, sz
    float Gimbal_XY_length; /// (sx^2 + sy^2)^1/2

    Yaw_Z_Rotation_Mat(f32DroneYaw_rad, &tDroneYaw_Mat);
    Pitch_Y_Rotation_Mat(f32DronePitch_rad, &tDronePitch_Mat);
    Roll_X_Rotation_Mat(f32DroneRoll_rad, &tDroneRoll_Mat);
    Yaw_Z_Rotation_Mat(f32GimbalYaw_rad, &tGimbalYaw_Mat);
    Pitch_Y_Rotation_Mat(f32GimbalPitch_rad, &tGimbalPitch_Mat);

    matrix_vector_multiply3_1(&tGimbalPitch_Mat, &X_basis_vec, &Cal_1_Vec);// theta z
    matrix_vector_multiply3_1(&tGimbalYaw_Mat, &Cal_1_Vec, &Cal_2_Vec);// psi z
    matrix_vector_multiply3_1(&tDroneRoll_Mat, &Cal_2_Vec, &Cal_3_Vec);// phi 
    matrix_vector_multiply3_1(&tDronePitch_Mat, &Cal_3_Vec, &Cal_4_Vec);// theta
    matrix_vector_multiply3_1(&tDroneYaw_Mat, &Cal_4_Vec, &tGimbalDirection_Vec);// psi

    f32GimbalDirection_Vec_Norm = vector_Euclid_norm3_1(&tGimbalDirection_Vec);
    Gimbal_X_direc = tGimbalDirection_Vec.f32Val[0][0];
    Gimbal_Y_direc = tGimbalDirection_Vec.f32Val[1][0];
    Gimbal_Z_direc = tGimbalDirection_Vec.f32Val[2][0];

  
    f32Ratio = f32DroneAltitude_m / Gimbal_Z_direc;
    /// Gimbal_Direction_Vec_Norm must be 1.0;
    /// Gimbal_Z_direc must be positive number, because Gimbal maybe look down
    /// if error,DO NOT update about Gimbal Direction Yaw & Angle
    if (f32GimbalDirection_Vec_Norm < 1.1 && f32GimbalDirection_Vec_Norm > 0.9) {
        if (Gimbal_Z_direc > 0.0) {
            Gimbal_XY_length = sqrtf(powf(Gimbal_X_direc, 2.0) + powf(Gimbal_Y_direc, 2.0));
            f32GimbalDirectionAngle_rad = atan2f(Gimbal_XY_length, Gimbal_Z_direc);
            f32GimbalDirectionYaw_rad = atan2f(Gimbal_Y_direc, Gimbal_X_direc);
        }
    }
}

int32_t vector_addition3_1(tVector3_1* tInput1, tVector3_1* tInput2, tVector3_1* tVector_add) {
    /////return 값으로 tVector3값을 내보낼것임. 
    ///// input으로 tInput이 저장되어있는 Pointer를 받을것임.
    ///// 해당 메모리 주소에 있는 f32Val을 기반으로 연산을 할것임.
    ///// output을 static으로 정의하지 않는이상, 계산결과가 함수 외부로 나가지 않는다.
    ///// 따라서 함수의 성공여부만을 알려주는 int형으로 만들어서 0,1을 반환시켜주고,
    ///// output pointer를 입력변수로 만들어서 저장시켜준다.
    int32_t vectorRow, vectorColumn;
    int32_t rVal = 0;
    int32_t intTestVal = 0;

    for (vectorRow = 0; vectorRow < 3; vectorRow++) {
        tVector_add->f32Val[vectorRow][0] = tInput1->f32Val[vectorRow][0] + tInput2->f32Val[vectorRow][0];
        intTestVal += tVector_add->f32Val[vectorRow][0];
    }
    if (isnan(intTestVal)) {
        rVal = 0;
    }
    else {
        rVal = 1;
    }
    return rVal;
}

int32_t vector_subtraction3_1(tVector3_1* tInput1, tVector3_1* tInput2, tVector3_1* tVector_sub) {
    int32_t vectorRow, vectorColumn;
    int32_t rVal = 0;
    int32_t intTestVal = 0;

    for (vectorRow = 0; vectorRow < 3; vectorRow++) {
        tVector_sub->f32Val[vectorRow][0] = tInput1->f32Val[vectorRow][0] - tInput2->f32Val[vectorRow][0];
        intTestVal += tVector_sub->f32Val[vectorRow][0];
    }
    if (isnan(intTestVal)) {
        rVal = 0;
    }
    else {
        rVal = 1;
    }
    return rVal;
}

int32_t vector_scalar_multiply3_1(tVector3_1* tInput1, float f32scalar, tVector3_1* tVector_scalar_multi) {
    int32_t vectorRow, vectorColumn;
    int32_t rVal = 0;
    int32_t intTestVal = 0;
    for (vectorRow = 0; vectorRow < 3; vectorRow++) {
        tVector_scalar_multi->f32Val[vectorRow][0] = tInput1->f32Val[vectorRow][0] * f32scalar;
        intTestVal += tVector_scalar_multi->f32Val[vectorRow][0];
    }
    if (isnan(intTestVal)) {
        rVal = 0;
    }
    else {
        rVal = 1;
    }
    return rVal;
}
int32_t matrix_vector_multiply3_1(tMatrix3_3* InputMat, tVector3_1* Inputvec, tVector3_1* tmat_vec_multi) {
    int32_t vectorRow, vectorColumn;
    int32_t rVal = 0;
    int32_t intTestVal = 0;
    tmat_vec_multi->f32Val[0][0] = 0.0;
    tmat_vec_multi->f32Val[1][0] = 0.0;
    tmat_vec_multi->f32Val[2][0] = 0.0;

    for (vectorRow = 0; vectorRow < 3; vectorRow++) {
        for (vectorColumn = 0; vectorColumn < 3; vectorColumn++) {
            tmat_vec_multi->f32Val[vectorRow][0] += InputMat->f32Val[vectorRow][vectorColumn] * Inputvec->f32Val[vectorColumn][0];
            intTestVal += tmat_vec_multi->f32Val[vectorRow][0];
        }
    }
    if (isnan(intTestVal)) {
        rVal = 0;
    }
    else {
        rVal = 1;
    }
    return rVal;
}
float vector_Euclid_norm3_1(tVector3_1* Inputvec) {
    int32_t vectorRow;
    float f32Vector_norm_square = 0.0;
    float f32Vector_norm = 0.0;
    for (vectorRow = 0; vectorRow < 3; vectorRow++) {
        f32Vector_norm_square += powf(Inputvec->f32Val[vectorRow][0], 2);
    }
    f32Vector_norm = sqrtf(f32Vector_norm_square);
    return f32Vector_norm;
}
int32_t matrix_matrix_multiply3_3(tMatrix3_3* FrontMat, tMatrix3_3* BehindMat, tMatrix3_3* OutMat) {
    int32_t vectorRow1, vectorRow2, vectorCol;//// component number 
    int32_t rVal = 0;
    int32_t intTestVal = 0;
    tVector3_1 BehindColVec, OutColVec;
    for (vectorCol = 0; vectorCol < 3; vectorCol++) {
        for (vectorRow1 = 0; vectorRow1 < 3; vectorRow1++) {
            BehindColVec.f32Val[vectorRow1][0] = BehindMat->f32Val[vectorRow1][vectorCol];
        }
        matrix_vector_multiply3_1(FrontMat, &BehindColVec, &OutColVec);
        for (vectorRow2 = 0; vectorRow2 < 3; vectorRow2++) {
            OutMat->f32Val[vectorRow2][vectorCol] = OutColVec.f32Val[vectorRow2][0];
            intTestVal += OutMat->f32Val[vectorRow2][vectorCol];
        }
    }
    if (isnan(intTestVal)) {
        rVal = 0;
    }
    else {
        rVal = 1;
    }
    return rVal;
}
void Yaw_Z_Rotation_Mat(float YawAngle_rad, tMatrix3_3* z_Rotation_Mat) {
    /// z_Rotationo_Mat = [ cos(Yaw) -sin(Yaw) 0
    ///                     sin(Yaw)  cos(Yaw) 0
    ///                           0         0  1]
    z_Rotation_Mat->f32Val[0][0] = cosf(YawAngle_rad);
    z_Rotation_Mat->f32Val[0][1] = -sinf(YawAngle_rad);
    z_Rotation_Mat->f32Val[0][2] = 0.0;
    z_Rotation_Mat->f32Val[1][0] = sinf(YawAngle_rad);
    z_Rotation_Mat->f32Val[1][1] = cosf(YawAngle_rad);
    z_Rotation_Mat->f32Val[1][2] = 0.0;
    z_Rotation_Mat->f32Val[2][0] = 0.0;
    z_Rotation_Mat->f32Val[2][1] = 0.0;
    z_Rotation_Mat->f32Val[2][2] = 1.0;
}
void Pitch_Y_Rotation_Mat(float PitchAngle_rad, tMatrix3_3* y_Rotation_Mat) {
    /// y_Rotation_Mat = [ cos(Pitch)   0   sin(Pitch)
    ///                             0   1           0       
    ///                   -sin(Pitch)   0   cos(Pitch)]
    y_Rotation_Mat->f32Val[0][0] = cosf(PitchAngle_rad);
    y_Rotation_Mat->f32Val[0][1] = 0.0;
    y_Rotation_Mat->f32Val[0][2] = sinf(PitchAngle_rad);
    y_Rotation_Mat->f32Val[1][0] = 0.0;
    y_Rotation_Mat->f32Val[1][1] = 1.0;
    y_Rotation_Mat->f32Val[1][2] = 0.0;
    y_Rotation_Mat->f32Val[2][0] = -sinf(PitchAngle_rad);
    y_Rotation_Mat->f32Val[2][1] = 0.0;
    y_Rotation_Mat->f32Val[2][2] = cosf(PitchAngle_rad);
}
void Roll_X_Rotation_Mat(float RollAngle_rad, tMatrix3_3* x_Rotation_Mat) {
    /// z_Rotation_Mat = [    1           0          0
    ///                       0   cos(Roll)   -sin(Roll)
    ///                       0   sin(Roll)   cos(Roll)]
    x_Rotation_Mat->f32Val[0][0] = 1.0;
    x_Rotation_Mat->f32Val[0][1] = 0.0;
    x_Rotation_Mat->f32Val[0][2] = 0.0;
    x_Rotation_Mat->f32Val[1][0] = 0.0;
    x_Rotation_Mat->f32Val[1][1] = cosf(RollAngle_rad);
    x_Rotation_Mat->f32Val[1][2] = -sinf(RollAngle_rad);
    x_Rotation_Mat->f32Val[2][0] = 0.0;
    x_Rotation_Mat->f32Val[2][1] = sinf(RollAngle_rad);
    x_Rotation_Mat->f32Val[2][2] = cosf(RollAngle_rad);
}
void Inv_Rotation_Mat(tMatrix3_3* Rotation_Mat, tMatrix3_3* inv_Rot_Mat) {
    int32_t vectorRow, vectorCol;
    /// Same to make transpose matrix
    for (vectorRow = 0; vectorRow < 3; vectorRow++) {
        for (vectorCol = 0; vectorCol < 3; vectorCol++) {
            inv_Rot_Mat->f32Val[vectorCol][vectorRow] = Rotation_Mat->f32Val[vectorRow][vectorCol];
        }
    }
}
