# pragma once
#include "config.hpp"

class LQR
{
protected:
    float LQRKcoeffs[40][6] =
    {
        /*Q = [80.00, 80.00, 80.00, 80.00, 400.00, 1.00, 400.00, 1.00, 4000.00, 40.00] R = [1.50, 1.50, 0.50, 0.50]*/
        /* a1 + a2*L_len + a3*R_len + a4*L_len^2 + a5*L_len*R_len + a6*R_len^2 */
        { 4.676010 , 15.068425, -12.940568, -20.355902, 5.587317, 12.177546}, 
        { 8.902456 , 21.650128, -25.052656, -30.687128, 14.659820, 23.706377}, 
        { 4.920318 , -10.496091, 5.020348, 11.256762, -2.641789, -5.680564}, 
        { 5.106451 , -11.531182, 5.608126, 12.098436, -3.103476, -6.294098}, 
        { 11.698813 , 89.234988, -27.060249, -46.255474, -4.916690, 33.255563}, 
        { 1.113866 , 12.441994, -4.154171, 1.597654, -2.889237, 5.569243}, 
        { 6.023456 , -14.010501, 44.663387, 10.669947, -29.671896, -36.592553}, 
        { 0.690758 , 0.012674, 5.236867, -2.433250, 3.325290, -2.614395}, 
        { 27.789126 , -44.986166, -36.294598, 26.573717, 35.644953, 26.501955}, 
        { 4.806914 , -6.204739, -7.742574, 1.879393, 7.390458, 6.218596}, 
        { 4.676010 , -12.940568, 15.068425, 12.177546, 5.587317, -20.355902}, 
        { 8.902456 , -25.052656, 21.650128, 23.706377, 14.659820, -30.687128}, 
        { -4.920318 , -5.020348, 10.496091, 5.680564, 2.641789, -11.256762}, 
        { -5.106451 , -5.608126, 11.531182, 6.294098, 3.103476, -12.098436}, 
        { 6.023456 , 44.663387, -14.010501, -36.592553, -29.671896, 10.669947}, 
        { 0.690758 , 5.236867, 0.012674, -2.614395, 3.325290, -2.433250}, 
        { 11.698813 , -27.060249, 89.234988, 33.255563, -4.916690, -46.255474}, 
        { 1.113866 , -4.154171, 12.441994, 5.569243, -2.889237, 1.597654}, 
        { 27.789126 , -36.294598, -44.986166, 26.501955, 35.644953, 26.573717}, 
        { 4.806914 , -7.742574, -6.204739, 6.218596, 7.390458, 1.879393}, 
        { -3.716821 , -6.582498, 15.706059, 19.013163, -10.653096, -15.939164}, 
        { -6.437387 , -11.052068, 28.768748, 31.349697, -20.083120, -28.870209}, 
        { 3.385760 , 9.408029, 5.426233, -14.939534, 7.065578, -6.763613}, 
        { 3.501905 , 10.755211, 6.127704, -16.684448, 8.532833, -7.532488}, 
        { -21.158044 , -44.086963, -7.606184, 39.681324, -81.132439, 19.325552}, 
        { -1.879684 , -5.819986, 1.195725, 1.905842, -6.126114, -1.043759}, 
        { 1.978002 , 31.955713, 45.253251, -47.256467, 83.902173, -32.077168}, 
        { 0.262513 , 1.787131, 1.718764, -1.011748, 6.162143, 2.715008}, 
        { 54.632362 , 95.493834, -39.014799, -96.529178, -16.465460, 38.534819}, 
        { 6.354763 , 14.461484, -4.978966, -12.557989, -3.936102, 4.266624}, 
        { -3.716821 , 15.706059, -6.582498, -15.939164, -10.653096, 19.013163}, 
        { -6.437387 , 28.768748, -11.052068, -28.870209, -20.083120, 31.349697}, 
        { -3.385760 , -5.426233, -9.408029, 6.763613, -7.065578, 14.939534}, 
        { -3.501905 , -6.127704, -10.755211, 7.532488, -8.532833, 16.684448}, 
        { 1.978002 , 45.253251, 31.955713, -32.077168, 83.902173, -47.256467}, 
        { 0.262513 , 1.718764, 1.787131, 2.715008, 6.162143, -1.011748}, 
        { -21.158044 , -7.606184, -44.086963, 19.325552, -81.132439, 39.681324}, 
        { -1.879684 , 1.195725, -5.819986, -1.043759, -6.126114, 1.905842}, 
        { 54.632362 , -39.014799, 95.493834, 38.534819, -16.465460, -96.529178}, 
        { 6.354763 , -4.978966, 14.461484, 4.266624, -3.936102, -12.557989}
    };

    float LQRKBuf[40] = {0};
    float LQROutBuf[4] = {0};
    float LQRXerrorBuf[10] = {0};

    float * LQRXRefX;
    float * LQRXObsX;

public:

    /* [Tl;Tr;Tpl;Tpr] = -K(X_obs - X_ref) */
    void LQRCal(float *Tout)
    {
        // 1. Calculate Error: X_err = X_obs - X_ref
        float err[10] = {0};
        for (int i=0; i<10; i++)
        {
            err[i] = this->LQRXObsX[i] - this->LQRXRefX[i];
        }    
        
        // 2. Calculate U = -K * X_err
        // Matrix multiplication: [4x10] * [10x1] = [4x1]
        for (int i=0; i<4; i++)
        {
            float temp = 0.0f;
            for (int j=0; j<10; j++)
            {
                temp += LQRKBuf[i*10+j]*err[j];
            }   
            Tout[i] = temp;
        }
            
    }

    /* 根据腿长更新使用的矩阵k */
    void refreshLQRK(float L_LegLenth, float R_LegLenth, bool ifOffground)
    {
        L_LegLenth = (L_LegLenth < LQR_MIN_LEN_CTRL) ? LQR_MIN_LEN_CTRL : L_LegLenth;
        L_LegLenth = (L_LegLenth > LQR_MAX_LEN_CTRL) ? LQR_MAX_LEN_CTRL : L_LegLenth;
        R_LegLenth = (R_LegLenth < LQR_MIN_LEN_CTRL) ? LQR_MIN_LEN_CTRL : R_LegLenth;
        R_LegLenth = (R_LegLenth > LQR_MAX_LEN_CTRL) ? LQR_MAX_LEN_CTRL : R_LegLenth;
        //保留两位小数
        L_LegLenth = roundf(L_LegLenth * 100) / 100.0f;
        R_LegLenth = roundf(R_LegLenth * 100) / 100.0f;

        /* a1 + a2*L_len + a3*R_len + a4*L_len^2 + a5*L_len*R_len + a6*R_len^2 */ 
        if (!ifOffground)
        {
            for(int i = 0; i < 40; i++)
            {
                LQRKBuf[i] = LQRKcoeffs[i][0] + LQRKcoeffs[i][1] * L_LegLenth + LQRKcoeffs[i][2] * R_LegLenth + LQRKcoeffs[i][3] * L_LegLenth * L_LegLenth + LQRKcoeffs[i][4] * L_LegLenth * R_LegLenth + LQRKcoeffs[i][5] * R_LegLenth * R_LegLenth;
            }
        }
        else
        {
            for(int i = 0; i < 40; i++)
            {
                LQRKBuf[i] = 0.0f;
            }
            for(int j = 24; j < 28; j++)
            {
                LQRKBuf[j] = LQRKcoeffs[j][0] + LQRKcoeffs[j][1] * L_LegLenth + LQRKcoeffs[j][2] * R_LegLenth + LQRKcoeffs[j][3] * L_LegLenth * L_LegLenth + LQRKcoeffs[j][4] * L_LegLenth * R_LegLenth + LQRKcoeffs[j][5] * R_LegLenth * R_LegLenth;
            }
            for(int k = 34; k < 38; k++)
            {
                LQRKBuf[k] = LQRKcoeffs[k][0] + LQRKcoeffs[k][1] * L_LegLenth + LQRKcoeffs[k][2] * R_LegLenth + LQRKcoeffs[k][3] * L_LegLenth * L_LegLenth + LQRKcoeffs[k][4] * L_LegLenth * R_LegLenth + LQRKcoeffs[k][5] * R_LegLenth * R_LegLenth;
            }
        }
    }

    // Modified to accept raw float pointers or extract data pointer from arm_matrix_instance_f32 if needed
    void InitMatX(float *pMatXRef, float *pMatXObs) 
    {
        // Store pointers to the actual data arrays
        this->LQRXRefX = pMatXRef;
        this->LQRXObsX = pMatXObs;
    }

};
