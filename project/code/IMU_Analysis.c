/********************************************************************************************************************
* 文件名称     IMU_Analysis.cpp
* 功能描述     IMU姿态解算和校准模块
* 使用说明     1. 支持三轴/六轴/九轴模式，通过 IMU_ANALYSIS_MODE 宏选择
*              2. 陀螺仪校准：采集静止时的数据，计算零偏
*              3. 磁力计校准：支持Min-Max法和椭球拟合法，旋转设备采集数据
*              4. 磁力计校准方法通过 MAG_CALIB_METHOD 宏选择（1:Min-Max, 2:椭球拟合）
********************************************************************************************************************/
#include "zf_common_headfile.h"


#include "IMU_Analysis.h"
#include <math.h>






/*******************************************************************************************************************/
/*[S] 通用全局变量 [S]-----------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// 全局变量
volatile float Yaw_Result = 0.0f;    // 偏航角（Yaw）
volatile float Roll_Result = 0.0f;   // 横滚角（Roll）
volatile float Pitch_Result = 0.0f;  // 俯仰角（Pitch）
// IMU 数据采集和分析使能标志位
volatile uint8_t IMU_D_and_A_Enable = 0;
/*******************************************************************************************************************/
/*-----------------------------------------------------------------------------------------------[E] 通用全局变量 [E]*/
/*******************************************************************************************************************/






/*======================================================*/
/*[数据读取]**********************************************/
/*======================================================*/

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU 初始化
// 备注信息     0 - 初始化成功; 1 - 初始化失败; 4 - imu被禁用 
//-------------------------------------------------------------------------------------------------------------------
uint8 IMU_init(void)
{
    #if   DEFINE_IMU_ANALYSIS_MODE != 0 
        uint8 imu_ret = imu963ra_init();
        return imu_ret;
    #else
        return 4;
    #endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU 获取原始数据
// 使用示例     IMU_Update_Data();                                              // 执行该函数后，直接查看对应的变量即可
// 备注信息     定时器定时中断定时调用该函数，获取 IMU963RA 原始数据
//-------------------------------------------------------------------------------------------------------------------
// IMU963RA 原始数据变量
// 加速度计原始值
// imu963ra_acc_x           imu963ra_acc_y          imu963ra_acc_z
// 陀螺仪原始值
// imu963ra_gyro_x          imu963ra_gyro_y         imu963ra_gyro_z
// 磁力计原始值
// imu963ra_mag_x           imu963ra_mag_y          imu963ra_mag_z  
void IMU_Update_Data(void)
{
    #if   DEFINE_IMU_ANALYSIS_MODE != 0 
        imu963ra_get_acc();
        imu963ra_get_gyro();
        imu963ra_get_mag();
    #endif
}
/*======================================================*/
/**********************************************[数据读取]*/
/*======================================================*/





/*======================================================*/
/*[加速度计]**********************************************/
/*======================================================*/

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     应用加速度计参数
// 使用示例     IMU_Acc_Apply(&ax, &ay, &az);
//-------------------------------------------------------------------------------------------------------------------
void IMU_Acc_Apply(float *ax, float *ay, float *az)
{
    *ax = (float)imu963ra_acc_x;
    *ay = (float)imu963ra_acc_y;
    *az = (float)imu963ra_acc_z;
}
/*======================================================*/
/**********************************************[加速度计]*/
/*======================================================*/


/*======================================================*/
/*[陀螺仪校准]********************************************/
/*======================================================*/

// 定义并初始化陀螺仪校准结构体
Gyro_Calib_StructDef gyro_cal = {
    .calib_state = GYRO_CALIB_STATE_IDLE, // 初始化为未校准

    .calib_count = 0,
    .offset_x = 0.0f,
    .offset_y = 0.0f,
    .offset_z = 0.0f,
    .sum_x = 0.0f,
    .sum_y = 0.0f,
    .sum_z = 0.0f,
};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     陀螺仪开始校准
// 使用示例     Gyro_Calib_Start(&gyro_cal);
//-------------------------------------------------------------------------------------------------------------------
void IMU_Gyro_Calib_Start(Gyro_Calib_StructDef *cal)
{
    cal->calib_state = GYRO_CALIB_STATE_RUNNING; // 设置状态为校准中

    cal->calib_count = 0;
    cal->sum_x = 0.0f;
    cal->sum_y = 0.0f;
    cal->sum_z = 0.0f;
    cal->offset_x = 0.0f;
    cal->offset_y = 0.0f;
    cal->offset_z = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     陀螺仪校准状态机
// 返回参数     校准状态：0-未校准，1-校准中，2-已校准
// 使用示例     Gyro_Calib_Check(&gyro_cal);
// 使用要求：
//   - 在静止状态下进行校准，确保设备不受外部运动干扰
//-------------------------------------------------------------------------------------------------------------------
int IMU_Gyro_Calib_Check(Gyro_Calib_StructDef *cal)
{
    if(cal->calib_state == GYRO_CALIB_STATE_DONE)
    {
        return 2; // 校准完
    }

    if(cal->calib_state == GYRO_CALIB_STATE_IDLE)
    {
        return 0; // 未校准
    }
    
    // 那就是状态机还是运行子状态
    if(IMU_D_and_A_Enable)
    {
        IMU_Update_Data();
        // 收集数据
        cal->sum_x += (float)imu963ra_gyro_x;
        cal->sum_y += (float)imu963ra_gyro_y;
        cal->sum_z += (float)imu963ra_gyro_z;

        cal->calib_count++;
        IMU_D_and_A_Enable = 1;
        
        // 样本数量达成目标
        if(cal->calib_count >= GYRO_CALIB_TARGET_SAMPLES)
        {
            // 计算陀螺仪浮点数偏移量
            cal->offset_x = (float)cal->sum_x / GYRO_CALIB_TARGET_SAMPLES;
            cal->offset_y = (float)cal->sum_y / GYRO_CALIB_TARGET_SAMPLES;
            cal->offset_z = (float)cal->sum_z / GYRO_CALIB_TARGET_SAMPLES;
            
            // 更新陀螺仪校准状态为完成
            cal->calib_state = GYRO_CALIB_STATE_DONE;
            
            printf("GYRO_CAL_OFFSET: X=%.2f, Y=%.2f, Z=%.2f\r\n", cal->offset_x, cal->offset_y, cal->offset_z);
        }
    }
    
    return 1; // 校准中
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     应用陀螺仪校准参数
// 参数说明     cal             陀螺仪校准结构体指针
// 参数说明     gx              用于存储校准后X轴数据的指针
// 参数说明     gy              用于存储校准后Y轴数据的指针
// 参数说明     gz              用于存储校准后Z轴数据的指针
// 使用示例     float gx, gy, gz;
// 使用示例     IMU_Gyro_Apply(&gyro_cal, &gx, &gy, &gz);
// 备注信息     函数内部直接使用全局变量 imu963ra_gyro_x/y/z 获取原始数据
//              校准完成后，将校准结果存储到传入的指针中
//              未校准时，应用默认的校准参数
//-------------------------------------------------------------------------------------------------------------------
void IMU_Gyro_Apply(Gyro_Calib_StructDef *cal, float *gx, float *gy, float *gz)
{
    // 如果校准完成，应用校准参数
    if (cal->calib_state == GYRO_CALIB_STATE_DONE)
    {
        *gx = (float)imu963ra_gyro_x - cal->offset_x;
        *gy = (float)imu963ra_gyro_y - cal->offset_y;
        *gz = (float)imu963ra_gyro_z - cal->offset_z;

        if (-7.0f < *gx && *gx < 7.0f){*gx = 0.0f;}
        if (-7.0f < *gy && *gy < 7.0f){*gy = 0.0f;}
        if (-7.0f < *gz && *gz < 7.0f){*gz = 0.0f;}
    }
    // 未校准时，应用默认的校准参数
    else
    {
        *gx = (float)imu963ra_gyro_x - 5.3f;
        *gy = (float)imu963ra_gyro_y + 5.9f;
        *gz = (float)imu963ra_gyro_z + 7.0f;

        if (-7.0f < *gx && *gx < 7.0f){*gx = 0.0f;}
        if (-7.0f < *gy && *gy < 7.0f){*gy = 0.0f;}
        if (-7.0f < *gz && *gz < 7.0f){*gz = 0.0f;}
    }
}
/*======================================================*/
/********************************************[陀螺仪校准]*/
/*======================================================*/






/*======================================================*/
/*[磁力计校准]********************************************/
/*======================================================*/

// 磁力计校准结构体变量
Mag_Calib_StructDef mag_cal = {
    .calib_state = MAG_CALIB_STATE_IDLE,
    .calib_count = 0,
    .offset_x = 0.0f,    // X轴偏移
    .offset_y = 0.0f,     // Y轴偏移
    .offset_z = 0.0f,     // Z轴偏移
    .scale_x = 1.0f,   // X轴缩放
    .scale_y = 1.0f,   // Y轴缩放
    .scale_z = 1.0f,   // Z轴缩放
    .max_x = -32768,
    .min_x = 32767,
    .max_y = -32768,
    .min_y = 32767,
    .max_z = -32768,
    .min_z = 32767,
};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     磁力计校准初始化
// 使用示例     Mag_Calib_Start(&mag_cal);
//-------------------------------------------------------------------------------------------------------------------
void IMU_Mag_Calib_Start(Mag_Calib_StructDef *cal)
{
    // 校准状态置为0，表示需要进行校准
    cal->calib_state = MAG_CALIB_STATE_RUNNING;
    cal->calib_count = 0;  
    // 硬铁偏移（中心点）
    cal->offset_x = 0.0f;     // X轴偏移
    cal->offset_y = 0.0f;     // Y轴偏移
    cal->offset_z = 0.0f;     // Z轴偏移    
    // 软铁缩放因子
    cal->scale_x = 1.0f;   // X轴缩放
    cal->scale_y = 1.0f;   // Y轴缩放
    cal->scale_z = 1.0f;   // Z轴缩放
    // 初始化最大最小值为当前测量范围
    cal->max_x = -32768;
    cal->min_x = 32767;
    cal->max_y = -32768;
    cal->min_y = 32767;
    cal->max_z = -32768;
    cal->min_z = 32767;
}

#if MAG_CALIB_METHOD == 1 // 应用Min-Max法
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     磁力计校准状态机(Min-Max法)
// 返回参数     校准状态：0-未校准，1-校准中，2-校准完
// 使用示例     Mag_Calib_Check(&mag_cal);
// 算法原理：
//   1. 采集各个方向的磁场数据
//   2. 找到每个轴的最大值和最小值
//   3. 硬铁偏移 = (最大值 + 最小值) / 2
//   4. 软铁缩放 = 最大半量程 / 当前轴半量程
// 
// 优点：
//   - 计算简单，速度快
//   - 内存占用小
// 缺点：
//   - 精度相对较低
//   - 对旋转的均匀性要求较高
// 
// 使用要求：
//   - 找一个远离强磁场干扰的环境（避开电机、磁铁、扬声器等）
//   - 保持设备稳定，避免剧烈晃动
//   - 缓慢旋转设备，确保覆盖以下所有方向：
//     1. 水平旋转：绕垂直轴360度旋转
//     2. 俯仰旋转：前端上下摆动，覆盖-90°到+90°
//     3. 横滚旋转：左右倾斜，覆盖-90°到+90°
//     4. 斜向旋转：进行一些对角线方向的旋转
//   - 旋转过程持续约20秒，确保采集足够的数据
//   - 校准完成后，会自动计算并输出校准参数
//-------------------------------------------------------------------------------------------------------------------
int IMU_Mag_Calib_Check(Mag_Calib_StructDef *cal)
{
    if(cal->calib_state == MAG_CALIB_STATE_DONE)
    {
        return 2; // 校准完
    }

    if(cal->calib_state == MAG_CALIB_STATE_IDLE)
    {
        return 0; // 未校准
    }

    // 那就是状态机还是运行子状态
    // 检查是否允许收集数据
    if (IMU_D_and_A_Enable)
    {
        IMU_Update_Data();

        // 更新最大最小值
        if (imu963ra_mag_x > cal->max_x) cal->max_x = imu963ra_mag_x;
        if (imu963ra_mag_x < cal->min_x) cal->min_x = imu963ra_mag_x;
        if (imu963ra_mag_y > cal->max_y) cal->max_y = imu963ra_mag_y;
        if (imu963ra_mag_y < cal->min_y) cal->min_y = imu963ra_mag_y;
        if (imu963ra_mag_z > cal->max_z) cal->max_z = imu963ra_mag_z;
        if (imu963ra_mag_z < cal->min_z) cal->min_z = imu963ra_mag_z;

        cal->calib_count++;
        IMU_D_and_A_Enable = 0;

        // 样本数量达成目标
        if (cal->calib_count >= MAG_CALIB_MINMAX_TARGET_SAMPLES)
        {
            // 计算硬铁偏移（椭球中心）
            cal->offset_x = (float)(cal->max_x + cal->min_x) / 2.0f;
            cal->offset_y = (float)(cal->max_y + cal->min_y) / 2.0f;
            cal->offset_z = (float)(cal->max_z + cal->min_z) / 2.0f;

            // 计算软铁缩放因子
            float half_range_x = (float)(cal->max_x - cal->min_x) / 2.0f;   
            float half_range_y = (float)(cal->max_y - cal->min_y) / 2.0f;
            float half_range_z = (float)(cal->max_z - cal->min_z) / 2.0f;
            
            float max_half_range = half_range_x;
            if (half_range_y > max_half_range) max_half_range = half_range_y;
            if (half_range_z > max_half_range) max_half_range = half_range_z;
            
            if (half_range_x > 1.0f) cal->scale_x = max_half_range / half_range_x;
            if (half_range_y > 1.0f) cal->scale_y = max_half_range / half_range_y;
            if (half_range_z > 1.0f) cal->scale_z = max_half_range / half_range_z;

            cal->calib_state = MAG_CALIB_STATE_DONE;
            printf("MAG_CAL_OFFSET: X=%.2f, Y=%.2f, Z=%.2f\r\n", cal->offset_x, cal->offset_y, cal->offset_z);
            printf("MAG_CAL_SCALE: X=%.2f, Y=%.2f, Z=%.2f\r\n", cal->scale_x, cal->scale_y, cal->scale_z);
        }
    }

    return 1; // 校准中
}
#endif

#if MAG_CALIB_METHOD == 2 // 应用椭球拟合法
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     磁力计校准状态机(椭球拟合法)
// 返回参数     校准状态：0-未校准，1-校准中，2-校准完
// 使用示例     Mag_Calib_Check(&mag_cal);
// 算法原理：
//   1. 采集3000个磁场数据点
//   2. 使用最小二乘法拟合椭球模型
//   3. 通过高斯消元法求解线性方程组
//   4. 计算椭球中心（硬铁偏移）和缩放因子（软铁校正）
// 
// 优点：
//   - 精度高，能处理复杂干扰
//   - 适应性强
// 缺点：
//   - 计算复杂
//   - 内存占用大（需要存储3000个数据点）
// 
// 容错机制：
//   - 当拟合失败时，自动切换到Min-Max法
// 使用要求：
//   - 找一个远离强磁场干扰的环境（避开电机、磁铁、扬声器等）
//   - 保持设备稳定，避免剧烈晃动
//   - 缓慢、均匀地旋转设备，确保覆盖所有空间方向：
//     - 水平方向：顺时针和逆时针各旋转360°
//     - 垂直方向：上下俯仰，从底部朝上到顶部朝上
//     - 倾斜方向：左右横滚，从左侧朝上到右侧朝上
//     - 斜向方向：进行一些复合角度的旋转
//   - 旋转过程持续约15-20秒，确保采集3000个数据点
//   - 如果拟合失败，会自动切换到Min-Max法作为备用
//   - 校准完成后，会自动计算并输出校准参数
//-------------------------------------------------------------------------------------------------------------------
int IMU_Mag_Calib_Check(Mag_Calib_StructDef *cal)
{
    if(cal->calib_state == MAG_CALIB_STATE_DONE)
    {
        return 2; // 校准完
    }

    if(cal->calib_state == MAG_CALIB_STATE_IDLE)
    {
        return 0; // 未校准
    }

    // 那就是状态机还是运行子状态
    // 检查是否允许收集数据
    if (IMU_D_and_A_Enable)
    {
        IMU_Update_Data();

        // 存储采集的磁力计数据
        static int16_t ellipsoid_mag_x_buf[3000];
        static int16_t ellipsoid_mag_y_buf[3000];
        static int16_t ellipsoid_mag_z_buf[3000];
        static int16_t max_x = -32768, min_x = 32767;
        static int16_t max_y = -32768, min_y = 32767;
        static int16_t max_z = -32768, min_z = 32767;

        // 记录最大值和最小值
        if (imu963ra_mag_x > max_x) max_x = imu963ra_mag_x;
        if (imu963ra_mag_x < min_x) min_x = imu963ra_mag_x;
        if (imu963ra_mag_y > max_y) max_y = imu963ra_mag_y;
        if (imu963ra_mag_y < min_y) min_y = imu963ra_mag_y;
        if (imu963ra_mag_z > max_z) max_z = imu963ra_mag_z;
        if (imu963ra_mag_z < min_z) min_z = imu963ra_mag_z;

        // 存储数据点
        if (cal->calib_count < 3000)
        {
            ellipsoid_mag_x_buf[cal->calib_count] = imu963ra_mag_x;
            ellipsoid_mag_y_buf[cal->calib_count] = imu963ra_mag_y;
            ellipsoid_mag_z_buf[cal->calib_count] = imu963ra_mag_z;
            cal->calib_count++;
        }

        IMU_D_and_A_Enable = 0;

        // 采满固定数量的点后执行椭球拟合
        if (cal->calib_count >= 3000)
        {
            // 更新结构体中的最大最小值
            cal->max_x = max_x;
            cal->min_x = min_x;
            cal->max_y = max_y;
            cal->min_y = min_y;
            cal->max_z = max_z;
            cal->min_z = min_z;

            // 执行椭球拟合
            double m_matrix[6][6 + 1];
            double solve[6];

            // 初始化矩阵
            memset(m_matrix, 0, sizeof(m_matrix));
            memset(solve, 0, sizeof(solve));

            // 构建矩阵
            for (uint16_t i = 0; i < cal->calib_count; i++)
            {
                double x = (double)ellipsoid_mag_x_buf[i];
                double y = (double)ellipsoid_mag_y_buf[i];
                double z = (double)ellipsoid_mag_z_buf[i];
                double V[7];

                V[0] = y * y;
                V[1] = z * z;
                V[2] = x;
                V[3] = y;
                V[4] = z;
                V[5] = 1.0;
                V[6] = -x * x;

                for (uint8_t row = 0; row < 6; row++)
                {
                    for (uint8_t col = 0; col < 6 + 1; col++)
                    {
                        m_matrix[row][col] += V[row] * V[col];
                    }
                }
            }

            // 归一化矩阵
            for (uint8_t row = 0; row < 6; row++)
            {
                for (uint8_t col = 0; col < 6 + 1; col++)
                {
                    m_matrix[row][col] /= (double)cal->calib_count;
                }
            }

            // 高斯消元法求解
            for (uint8_t k = 0; k < 6; k++)
            {
                // 寻找主元素
                uint8_t max_row = k;
                for (uint8_t i = k + 1; i < 6; i++)
                {
                    if (fabs(m_matrix[i][k]) > fabs(m_matrix[max_row][k]))
                    {
                        max_row = i;
                    }
                }

                // 交换行
                if (max_row != k)
                {
                    for (uint8_t j = 0; j <= 6; j++)
                    {
                        double tmp = m_matrix[k][j];
                        m_matrix[k][j] = m_matrix[max_row][j];
                        m_matrix[max_row][j] = tmp;
                    }
                }

                // 检查主元素是否为零
                if (fabs(m_matrix[k][k]) < 1e-10)
                {
                    // 拟合失败，使用Min-Max法作为备用
                    cal->offset_x = (float)(cal->max_x + cal->min_x) / 2.0f;
                    cal->offset_y = (float)(cal->max_y + cal->min_y) / 2.0f;
                    cal->offset_z = (float)(cal->max_z + cal->min_z) / 2.0f;

                    float half_range_x = (float)(cal->max_x - cal->min_x) / 2.0f;
                    float half_range_y = (float)(cal->max_y - cal->min_y) / 2.0f;
                    float half_range_z = (float)(cal->max_z - cal->min_z) / 2.0f;
                    
                    float max_half_range = half_range_x;
                    if (half_range_y > max_half_range) max_half_range = half_range_y;
                    if (half_range_z > max_half_range) max_half_range = half_range_z;
                    
                    if (half_range_x > 1.0f) cal->scale_x = max_half_range / half_range_x;
                    if (half_range_y > 1.0f) cal->scale_y = max_half_range / half_range_y;
                    if (half_range_z > 1.0f) cal->scale_z = max_half_range / half_range_z;
                    
                    cal->calib_state = MAG_CALIB_STATE_DONE;
                    printf("MAG_CAL_OFFSET: X=%.2f, Y=%.2f, Z=%.2f\r\n", cal->offset_x, cal->offset_y, cal->offset_z);
                    printf("MAG_CAL_SCALE: X=%.2f, Y=%.2f, Z=%.2f\r\n", cal->scale_x, cal->scale_y, cal->scale_z);
                    return 2;
                }

                // 消元
                for (uint8_t i = k + 1; i < 6; i++)
                {
                    double factor = m_matrix[i][k] / m_matrix[k][k];
                    for (uint8_t j = k; j <= 6; j++)
                    {
                        m_matrix[i][j] -= factor * m_matrix[k][j];
                    }
                }
            }

            // 回代求解
            for (int8_t i = 5; i >= 0; i--)
            {
                double sum = 0.0;
                for (uint8_t j = i + 1; j < 6; j++)
                {
                    sum += m_matrix[i][j] * solve[j];
                }
                solve[i] = (m_matrix[i][6] - sum) / m_matrix[i][i];
            }

            // 计算校准参数
            double a = solve[0];
            double b = solve[1];
            double c = solve[2];
            double d = solve[3];
            double e = solve[4];
            double f = solve[5];

            double X0 = -c / 2.0;
            double Y0 = -d / (2.0 * a);
            double Z0 = -e / (2.0 * b);
            double temp = X0 * X0 + a * Y0 * Y0 + b * Z0 * Z0 - f;

            if (temp > 0.0)
            {
                double A = sqrt(temp);
                double B = A / sqrt(a);
                double C = A / sqrt(b);

                cal->offset_x = (float)X0;
                cal->offset_y = (float)Y0;
                cal->offset_z = (float)Z0;

                cal->scale_x = (float)(1.0 / A);
                cal->scale_y = (float)(1.0 / B);
                cal->scale_z = (float)(1.0 / C);
            }
            else
            {
                // 拟合失败，使用Min-Max法作为备用
                cal->offset_x = (float)(cal->max_x + cal->min_x) / 2.0f;
                cal->offset_y = (float)(cal->max_y + cal->min_y) / 2.0f;
                cal->offset_z = (float)(cal->max_z + cal->min_z) / 2.0f;

                float half_range_x = (float)(cal->max_x - cal->min_x) / 2.0f;
                float half_range_y = (float)(cal->max_y - cal->min_y) / 2.0f;
                float half_range_z = (float)(cal->max_z - cal->min_z) / 2.0f;
                
                float max_half_range = half_range_x;
                if (half_range_y > max_half_range) max_half_range = half_range_y;
                if (half_range_z > max_half_range) max_half_range = half_range_z;
                
                if (half_range_x > 1.0f) cal->scale_x = max_half_range / half_range_x;
                if (half_range_y > 1.0f) cal->scale_y = max_half_range / half_range_y;
                if (half_range_z > 1.0f) cal->scale_z = max_half_range / half_range_z;
            }

            cal->calib_state = MAG_CALIB_STATE_DONE;
            printf("MAG_CAL_OFFSET: X=%.2f, Y=%.2f, Z=%.2f\r\n", cal->offset_x, cal->offset_y, cal->offset_z);
            printf("MAG_CAL_SCALE: X=%.6f, Y=%.6f, Z=%.6f\r\n", cal->scale_x, cal->scale_y, cal->scale_z);
            return 2;
        }
    }
    return 1; // 校准进行中
}
#endif

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     应用磁力计校准参数
// 参数说明     cal             磁力计校准结构体指针
// 参数说明     mx              用于存储校准后X轴数据的指针
// 参数说明     my              用于存储校准后Y轴数据的指针
// 参数说明     mz              用于存储校准后Z轴数据的指针
// 使用示例     int16_t mx, my, mz;
// 使用示例     IMU_Mag_Apply(&mag_cal, &mx, &my, &mz);
// 备注信息     函数内部直接使用全局变量 imu963ra_mag_x/y/z 获取原始数据
//              校准完成后，将校准结果存储到传入的指针中
//              未校准时，直接将原始数据存储到传入的指针中
//-------------------------------------------------------------------------------------------------------------------
void IMU_Mag_Apply(Mag_Calib_StructDef *cal, int16_t *mx, int16_t *my, int16_t *mz)
{
    // 如果校准完成，应用校准参数
    if (cal->calib_state == MAG_CALIB_STATE_DONE)
    {
        *mx = (int16_t)(((float)imu963ra_mag_x - cal->offset_x) * cal->scale_x);
        *my = (int16_t)(((float)imu963ra_mag_y - cal->offset_y) * cal->scale_y);
        *mz = (int16_t)(((float)imu963ra_mag_z - cal->offset_z) * cal->scale_z);
    }
    // 未校准时，直接将原始数据存储到传入的指针中
    else
    {
        *mx = (int16_t)imu963ra_mag_x;
        *my = (int16_t)imu963ra_mag_y;
        *mz = (int16_t)imu963ra_mag_z;
    }
}
/*======================================================*/
/********************************************[磁力计校准]*/
/*======================================================*/







/*======================================================*/
/*[姿态解算]**********************************************/
/*======================================================*/



// ”通用货“
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     快速平方根倒数
// 参数说明     x               输入值
// 返回参数     float           输出值 (1/sqrt(x))
// 使用示例     inv_sqrt = fast_sqrt(norm);
// 备注信息     Quake III 经典算法
//-------------------------------------------------------------------------------------------------------------------
float fast_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;

    int32_t i;
    memcpy(&i, &y, sizeof(int32_t));
    i = 0x5f3759df - (i >> 1);
    memcpy(&y, &i, sizeof(int32_t));
    y = y * (1.5f - (halfx * y * y));
    return y;
}



#if DEFINE_IMU_ANALYSIS_MODE == 1      
/*******************************************************************************************************************/
/*[S] [仅输出Yaw]三轴 [S]-------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

float CYCLE_T                       =0.01f;                              // 三轴四元数采样周期
#define PI                          3.14159265358979f

static float      q_3dof[4] = {1.0f, 0.0f, 0.0f, 0.0f};                     // 初始单位四元数（三轴解算专用）

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     基于纯角速度积分计算 Yaw (三轴解算方式)
// 参数说明     void
// 返回参数     void
// 使用示例     imu_transform_gyro();
// 备注信息     仅用于特殊调试或对比测试
//-------------------------------------------------------------------------------------------------------------------
void imu_transform_gyro(void)
{
    // 定义临时存储变量
    float gx_temp;
    float gy_temp;
    float gz_temp;

    // 通过统一接口获取原始数据
    IMU_Gyro_Apply(&gyro_cal, &gx_temp, &gy_temp, &gz_temp);

    float gx = imu963ra_gyro_transition(gx_temp / 10 * 10) * PI / 180.0f;
    float gy = imu963ra_gyro_transition(gy_temp / 10 * 10) * PI / 180.0f;
    float gz = imu963ra_gyro_transition(gz_temp / 10 * 10) * PI / 180.0f;

    // 暂存当前三轴四元数
    float q0 = q_3dof[0];
    float q1 = q_3dof[1];
    float q2 = q_3dof[2];
    float q3 = q_3dof[3];

    // 三轴四元数微分方程积分
    float dt = CYCLE_T;
    q_3dof[0] += (-q1 * gx - q2 * gy - q3 * gz) * 0.5f * dt;
    q_3dof[1] += ( q0 * gx + q2 * gz - q3 * gy) * 0.5f * dt;
    q_3dof[2] += ( q0 * gy - q1 * gz + q3 * gx) * 0.5f * dt;
    q_3dof[3] += ( q0 * gz + q1 * gy - q2 * gx) * 0.5f * dt;

    // 三轴四元数归一化
    float norm = sqrtf(q_3dof[0] * q_3dof[0] + q_3dof[1] * q_3dof[1] + q_3dof[2] * q_3dof[2] + q_3dof[3] * q_3dof[3]);
    if (norm > 0.0f)
    {
        norm = 1.0f / norm;
        q_3dof[0] *= norm;
        q_3dof[1] *= norm;
        q_3dof[2] *= norm;
        q_3dof[3] *= norm;
    }

    // 计算角度（基于三轴四元数）
    // 赋值给项目统一的Yaw解算结果变量
    Yaw_Result = atan2f(2.0f * q_3dof[1] * q_3dof[2] + 2.0f * q_3dof[0] * q_3dof[3], -2.0f * q_3dof[2] * q_3dof[2] - 2.0f * q_3dof[3] * q_3dof[3] + 1.0f) * 57.2957f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将当前三轴解算的 Yaw 归零 (不改变俯仰/横滚相对姿态)
// 参数说明     void
// 返回参数     void
// 使用示例     imu_zero_yaw();
// 备注信息     通过四元数逆旋转实现
//-------------------------------------------------------------------------------------------------------------------
void imu_zero_yaw(void)
{
    float q0 = q_3dof[0];
    float q1 = q_3dof[1];
    float q2 = q_3dof[2];
    float q3 = q_3dof[3];

    float yaw = atan2f(2.0f * (q1 * q2 + q0 * q3), -2.0f * (q2 * q2 + q3 * q3) + 1.0f);

    // 构造逆 yaw 旋转并左乘到当前四元数
    float half = -0.5f * yaw;
    float cz = cosf(half);
    float sz = sinf(half);

    float nq0 = cz * q0 - sz * q3;
    float nq1 = cz * q1 + sz * q2;
    float nq2 = cz * q2 - sz * q1;
    float nq3 = cz * q3 + sz * q0;

    // 归一化
    float norm = sqrtf(nq0 * nq0 + nq1 * nq1 + nq2 * nq2 + nq3 * nq3);
    if (norm > 0.0f)
    {
        norm = 1.0f / norm;
        q_3dof[0] = nq0 * norm;
        q_3dof[1] = nq1 * norm;
        q_3dof[2] = nq2 * norm;
        q_3dof[3] = nq3 * norm;
    }

    Yaw_Result = 0.0f;
}
/*******************************************************************************************************************/
/*-------------------------------------------------------------------------------------------[E] [仅输出Yaw]三轴 [E]*/
/*******************************************************************************************************************/
#endif


#if DEFINE_IMU_ANALYSIS_MODE == 2
/*******************************************************************************************************************/
/*[S] [仅输出Yaw]六轴 [S]-------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

imu_param_t imu_data_t = {
    .gyro_x = 0.0f, .gyro_y = 0.0f, .gyro_z = 0.0f,
    .acc_x = 0.0f, .acc_y = 0.0f, .acc_z = 0.0f,
    .pitch = 0.0f, .roll = 0.0f, .yaw = 0.0f
};

#define PI                          3.14159265358979f
#define YAW_DEADZONE_THRESHOLD      0.00005f                                // Yaw角单次更新死区阈值
#define DELTA_T                     0.0010055f                              // 六轴四元数采样时间

// 比例系数
//#define YAW_SCALE                   1.1842f                                 // Yaw角的比例系数
#define YAW_SCALE                     1.1465f                                 // Yaw角的比例系数

quater_param_t    Q_info = {1.0f, 0.0f, 0.0f, 0.0f};                        // 六轴位姿四元数

float             param_Kp = 0.0001f;                                       // 加速度计收敛速率比例增益
float             param_Ki = -0.0000324f;                                   // 陀螺仪收敛速率积分增益

static float      I_ex = 0.0f, I_ey = 0.0f, I_ez = 0.0f;                    // 误差积分项
static float      vx = 0.0f, vy = 0.0f, vz = 0.0f;                          // 机体坐标系上的重力单位向量

static float      last_raw_yaw = 0.0f;                                      // 上一次的原始 Yaw 角
static float      accumulated_yaw_drift = 0.0f;                             // 累积的死区误差
static bool       yaw_first_run = true;                                     // 第一次运行标志



//-------------------------------------------------------------------------------------------------------------------
// 函数简介     互补滤波 AHRS 姿态更新算法
// 参数说明     gx, gy, gz      陀螺仪角速度 (rad/s)
// 参数说明     ax, ay, az      加速度计数据 (m/s^2)
// 返回参数     void
// 使用示例     IMU_AHRSupdate(gx, gy, gz, ax, ay, az);
// 备注信息     更新全局四元数 Q_info
//-------------------------------------------------------------------------------------------------------------------
void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
    float halfT = 0.5f * DELTA_T;
    float ex, ey, ez;
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q1q1 = q1 * q1;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    float norm = fast_sqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    vx = 2.0f * (q1q3 - q0q2);
    vy = 2.0f * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    I_ex += halfT * ex;
    I_ey += halfT * ey;
    I_ez += halfT * ez;

    gx = gx + param_Kp * ex + param_Ki * I_ex;
    gy = gy + param_Kp * ey + param_Ki * I_ey;
    gz = gz + param_Kp * ez + param_Ki * I_ez;

    gz *= YAW_SCALE;

    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

    norm = fast_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    Q_info.q0 = q0 * norm;
    Q_info.q1 = q1 * norm;
    Q_info.q2 = q2 * norm;
    Q_info.q3 = q3 * norm;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取欧拉角 (基于六轴互补滤波)
// 参数说明     void
// 返回参数     void
// 使用示例     IMU_getEulerianAngles();
// 备注信息     包含 Yaw 角死区滤波处理
//-------------------------------------------------------------------------------------------------------------------
void IMU_getEulerianAngles(void)
{
    IMU_Acc_Apply(&imu_data_t.acc_x, &imu_data_t.acc_y, &imu_data_t.acc_z);
    IMU_Gyro_Apply(&gyro_cal, &imu_data_t.gyro_x, &imu_data_t.gyro_y, &imu_data_t.gyro_z);

     // 陀螺仪角速度转弧度
    imu_data_t.gyro_x = (-imu_data_t.gyro_x) * PI / 180.0f;
    imu_data_t.gyro_y = (imu_data_t.gyro_y) * PI / 180.0f;
    imu_data_t.gyro_z = (-imu_data_t.gyro_z) * PI / 180.0f;

    IMU_AHRSupdate(imu_data_t.gyro_x, imu_data_t.gyro_y, imu_data_t.gyro_z, imu_data_t.acc_x, imu_data_t.acc_y, imu_data_t.acc_z);

    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;

    // 计算当前的原始Yaw角
    float raw_yaw = atan2f(2.0f * q1 * q2 + 2.0f * q0 * q3, -2.0f * q2 * q2 - 2.0f * q3 * q3 + 1.0f) * 180.0f / PI;

    if (yaw_first_run)
    {
        last_raw_yaw = raw_yaw;
        yaw_first_run = false;
    }

    // 计算两次Yaw角之间的变化值
    float diff = raw_yaw - last_raw_yaw;

    // 处理角度越界跳变
    if (diff > 180.0f)       diff -= 360.0f;
    else if (diff < -180.0f) diff += 360.0f;

    // 如果变化值小于死区阈值，认为是噪声或漂移，进行累积以便后续减除
    if (fabsf(diff) < YAW_DEADZONE_THRESHOLD)
    {
        accumulated_yaw_drift += diff;
    }

    last_raw_yaw = raw_yaw;

    // 减去累积的死区误差
    float Daty_Z = raw_yaw - accumulated_yaw_drift;

    // 归一化到 [-180, 180]
    while (Daty_Z > 180.0f)  Daty_Z -= 360.0f;
    while (Daty_Z < -180.0f) Daty_Z += 360.0f;

    imu_data_t.yaw = Daty_Z;

    // 赋值给项目统一的Yaw解算结果变量
    Yaw_Result = Daty_Z;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将当前六轴解算的 Yaw 归零 (与三轴效果一致)
// 参数说明     void
// 返回参数     void
// 使用示例     ICM_zero_yaw();
// 备注信息     包含死区滤波变量重置
//-------------------------------------------------------------------------------------------------------------------
void ICM_zero_yaw(void)
{
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;

    float yaw = atan2f(2.0f * (q1 * q2 + q0 * q3), -2.0f * (q2 * q2 + q3 * q3) + 1.0f);

    float half = -0.5f * yaw;
    float cz = cosf(half);
    float sz = sinf(half);

    float nq0 = cz * q0 - sz * q3;
    float nq1 = cz * q1 + sz * q2;
    float nq2 = cz * q2 - sz * q1;
    float nq3 = cz * q3 + sz * q0;

    float norm = sqrtf(nq0 * nq0 + nq1 * nq1 + nq2 * nq2 + nq3 * nq3);
    if (norm > 0.0f)
    {
        norm = 1.0f / norm;
        Q_info.q0 = nq0 * norm;
        Q_info.q1 = nq1 * norm;
        Q_info.q2 = nq2 * norm;
        Q_info.q3 = nq3 * norm;
    }

    imu_data_t.yaw = 0.0f;
    // 赋值给项目统一的Yaw解算结果变量
    Yaw_Result = 0.0f;

    // 重置死区滤波相关的变量
    last_raw_yaw = 0.0f;
    accumulated_yaw_drift = 0.0f;
    yaw_first_run = true;
}

/*******************************************************************************************************************/
/*-------------------------------------------------------------------------------------------[E] [仅输出Yaw]六轴 [E]*/
/*******************************************************************************************************************/
#endif


#if DEFINE_IMU_ANALYSIS_MODE == 3
/*******************************************************************************************************************/
/*[S] [仅输出Yaw]九轴融合解算 [S]-------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

int16_t imu_quick_count = 0; // IMU稳定计数器
bool imu_stable = false; // IMU稳定标志

Mahony_AHRS_StructDef Mahony_ahrs = {
    // 初始化磁偏角（度）
    .mag_declination = 0.0f,

    // 初始化四元数（单位四元数）
    .q0 = 1.0f,
    .q1 = 0.0f,
    .q2 = 0.0f,
    .q3 = 0.0f,

    // 初始化PID参数
    .Kp = 10.0f,     // 比例增益
    .Ki = 0.005f,   // 积分增益

    .quick_kp = 30.0f,   // 快速校正比例增益
    .quick_ki = 0.005f,   // 快速校正积分增益

    // 初始化积分项
    .exInt = 0.0f,
    .eyInt = 0.0f,
    .ezInt = 0.0f
}; // Mahony算法结构体实例

//  函数简介     IMU初始化函数
// 参数说明     imu             IMU结构体指针
// 返回参数     void
// 使用示例     Mahony_AHRS_StructDef imu; Mahony_AHRS_Init(&imu);
// 备注信息     初始化四元数、PID参数和积分项
//-------------------------------------------------------------------------------------------------------------------
void Mahony_AHRS_Init(Mahony_AHRS_StructDef *imu)
{
    // 初始化磁偏角（度）
    imu->mag_declination = 0.0f;

    // 初始化四元数（单位四元数）
    imu->q0 = 1.0f;
    imu->q1 = 0.0f;
    imu->q2 = 0.0f;
    imu->q3 = 0.0f;

    // 初始化PID参数
    imu->Kp = 10.0f;     // 比例增益
    imu->Ki = 0.005f;   // 积分增益

    imu->quick_kp = 30.0f;   // 快速校正比例增益
    imu->quick_ki = 0.005f;   // 快速校正积分增益

    // 初始化积分项
    imu->exInt = 0.0f;
    imu->eyInt = 0.0f;
    imu->ezInt = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     Mahony九轴偏航角解算（保留原接口）
// 参数说明     dt              计算周期（秒）
// 返回参数     float           偏航角（度）
// 备注信息     使用原始数据处理链路并执行Mahony融合
//-------------------------------------------------------------------------------------------------------------------
static float Mahony_AHRS_Update(float dt)
{
    float gx, gy, gz;
    float ax, ay, az;
    int16_t mx, my, mz;
    float recipNorm;

    float q0 = Mahony_ahrs.q0;
    float q1 = Mahony_ahrs.q1;
    float q2 = Mahony_ahrs.q2;
    float q3 = Mahony_ahrs.q3;

    float q0q0, q0q1, q0q2, q0q3;
    float q1q1, q1q2, q1q3;
    float q2q2, q2q3;
    float q3q3;

    float vx, vy, vz;
    float wx, wy, wz;
    float hx, hy, bx, bz;
    float ex, ey, ez;
    float halfT;
    float yaw_deg;
    float kp;
    float ki;

    if (dt <= 1e-6f)
    {
        dt = 0.01f;
    }

    // imu_stable=0 使用快速参数，imu_stable=1 使用正常参数
    kp = imu_stable ? Mahony_ahrs.Kp : Mahony_ahrs.quick_kp;
    ki = imu_stable ? Mahony_ahrs.Ki : Mahony_ahrs.quick_ki;

    IMU_Acc_Apply(&ax, &ay, &az);
    IMU_Gyro_Apply(&gyro_cal, &gx, &gy, &gz);
    IMU_Mag_Apply(&mag_cal, &mx, &my, &mz);


    float declination_rad = Mahony_ahrs.mag_declination * 3.1415926535f / 180.0f;
    float mx_raw = mx;
    float my_raw = my;
    mx = mx_raw * cosf(declination_rad) - my_raw * sinf(declination_rad);
    my = mx_raw * sinf(declination_rad) + my_raw * cosf(declination_rad);

    recipNorm = fast_sqrt(ax * ax + ay * ay + az * az);
    if (recipNorm < 1e-6f)
    {
        return Yaw_Result;
    }
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    recipNorm = fast_sqrt(mx * mx + my * my + mz * mz);
    if (recipNorm < 1e-6f)
    {
        return Yaw_Result;
    }
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    vx = 2.0f * (q1q3 - q0q2);
    vy = 2.0f * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
    wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
    wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);

    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    if (ki > 0.0f)
    {
        Mahony_ahrs.exInt += ex * ki * dt;
        Mahony_ahrs.eyInt += ey * ki * dt;
        Mahony_ahrs.ezInt += ez * ki * dt;
    }
    else
    {
        Mahony_ahrs.exInt = 0.0f;
        Mahony_ahrs.eyInt = 0.0f;
        Mahony_ahrs.ezInt = 0.0f;
    }

    gx += kp * ex + Mahony_ahrs.exInt;
    gy += kp * ey + Mahony_ahrs.eyInt;
    gz += kp * ez + Mahony_ahrs.ezInt;

    halfT = 0.5f * dt;
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 += ( q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 += ( q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 += ( q0 * gz + q1 * gy - q2 * gx) * halfT;

    recipNorm = fast_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    Mahony_ahrs.q0 = q0;
    Mahony_ahrs.q1 = q1;
    Mahony_ahrs.q2 = q2;
    Mahony_ahrs.q3 = q3;

    yaw_deg = atan2f(2.0f * (q1 * q2 + q0 * q3),q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180.0f / 3.1415926535f;

    return yaw_deg;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     Mahony AHRS 算法重置函数
// 参数说明     imu             Mahony算法结构体指针
// 返回参数     void
// 使用示例     Mahony_AHRS_Reset(&Mahony_ahrs);
// 备注信息     将四元数重置为单位四元数，清零误差积分项和通用状态
//-------------------------------------------------------------------------------------------------------------------
void Mahony_AHRS_Reset(Mahony_AHRS_StructDef *imu)
{
    // 四元数重置为单位四元数
    imu->q0 = 1.0f;
    imu->q1 = 0.0f;
    imu->q2 = 0.0f;
    imu->q3 = 0.0f;
    
    // 误差积分项清零
    imu->exInt = 0.0f;
    imu->eyInt = 0.0f;
    imu->ezInt = 0.0f;

    // 通用状态重置
    imu_stable = false;           // 重置稳定标志
    imu_quick_count = 0;          // 重置稳定计数器
    Yaw_Result = 0.0f;            // 重置Yaw结果输出
}

/*******************************************************************************************************************/
/*-------------------------------------------------------------------------------------[E] [仅输出Yaw]九轴融合解算 [E]*/
/*******************************************************************************************************************/
#endif


#if DEFINE_IMU_ANALYSIS_MODE == 4
/*******************************************************************************************************************/
/*[S] [仅输出Yaw]只使用磁力计解算 [S]---------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

int16_t imu_quick_count = 0; // IMU稳定计数器
bool imu_stable = false; // IMU稳定标志

Mag_Get_Yaw_StructDef Mag_ahrs = {
    .mag_declination = -98.0f,
    .yaw_filter_alpha = 0.3f,
    .yaw_filtered = 0.0f
};// 磁力计获取偏航角结构体实例

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     三轴yaw参数初始化函数
// 参数说明     imu             三轴参数结构体指针
// 返回参数     void
// 使用示例     Mag_Get_Yaw_StructDef imu3axis; Mag_Get_Yaw_Init(&imu3axis);
// 备注信息     初始化磁偏角与yaw滤波参数
//-------------------------------------------------------------------------------------------------------------------
void Mag_Get_Yaw_Init(Mag_Get_Yaw_StructDef *imu)
{
    imu->mag_declination = -98.0f;
    imu->yaw_filter_alpha = 0.3f;
    imu->yaw_filtered = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     三轴偏航角解算（单一入口）
// 参数说明     void
// 返回参数     float           偏航角（度）
// 备注信息     仅使用磁力计数据计算偏航角
//-------------------------------------------------------------------------------------------------------------------
static float Mag_Get_Yaw_Update(void)
{
    // 读取加速度计数据用于倾斜补偿
    float ax;
    float ay;
    float az;
    // 读取磁力计数据
    int16_t mx;
    int16_t my;
    int16_t mz;

    IMU_Acc_Apply(&ax, &ay, &az);
    IMU_Mag_Apply(&mag_cal, &mx, &my, &mz);

    // 加速度计数据归一化
    float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (acc_norm < 1e-6f) return Mag_ahrs.yaw_filtered;
    ax /= acc_norm;
    ay /= acc_norm;
    az /= acc_norm;

    // 磁力计数据归一化
    float mag_norm = sqrtf(mx * mx + my * my + mz * mz);
    if (mag_norm < 1e-6f) return Mag_ahrs.yaw_filtered;
    mx /= mag_norm;
    my /= mag_norm;
    mz /= mag_norm;

    // 使用加速度构造仅含横滚/俯仰的四元数，将磁力计旋转到水平坐标系
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    float cr = cosf(0.5f * roll);
    float sr = sinf(0.5f * roll);
    float cp = cosf(0.5f * pitch);
    float sp = sinf(0.5f * pitch);

    // yaw=0 的 ZYX 欧拉角转四元数（机体系 -> 水平系）
    float q0 = cr * cp;
    float q1 = sr * cp;
    float q2 = cr * sp;
    float q3 = -sr * sp;

    // 旋转矩阵第一、二行即可得到水平面分量
    float r11 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    float r12 = 2.0f * (q1 * q2 - q0 * q3);
    float r13 = 2.0f * (q1 * q3 + q0 * q2);
    float r21 = 2.0f * (q1 * q2 + q0 * q3);
    float r22 = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
    float r23 = 2.0f * (q2 * q3 - q0 * q1);

    float mx_h = r11 * mx + r12 * my + r13 * mz;
    float my_h = r21 * mx + r22 * my + r23 * mz;

    // 检查水平分量有效性
    float mag_h_norm = sqrtf(mx_h * mx_h + my_h * my_h);
    if (mag_h_norm < 1e-6f) return Mag_ahrs.yaw_filtered;

    // 水平分量归一化
    mx_h /= mag_h_norm;
    my_h /= mag_h_norm;

    // 计算偏航角
    float yaw_rad = atan2f(my_h, mx_h);
    float yaw_deg = yaw_rad * 180.0f / 3.1415926535f;

    // 应用磁偏角
    yaw_deg += Mag_ahrs.mag_declination;

    // 归一化到 -180 到 180 度范围
    float yaw_mag = -yaw_deg;
    while (yaw_mag > 180.0f) yaw_mag -= 360.0f;
    while (yaw_mag < -180.0f) yaw_mag += 360.0f;
    
    return -yaw_mag;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     磁力计Yaw解算重置函数
// 参数说明     imu             磁力计Yaw结构体指针
// 返回参数     void
// 使用示例     Mag_Get_Yaw_Reset(&Mag_ahrs);
// 备注信息     重置滤波输出和通用状态
//-------------------------------------------------------------------------------------------------------------------
void Mag_Get_Yaw_Reset(Mag_Get_Yaw_StructDef *imu)
{
    // 滤波输出重置
    imu->yaw_filtered = 0.0f;

    // 通用状态重置
    imu_stable = false;           // 重置稳定标志
    imu_quick_count = 0;          // 重置稳定计数器
    Yaw_Result = 0.0f;            // 重置Yaw结果输出
}

/*******************************************************************************************************************/
/*---------------------------------------------------------------------------------[E] [仅输出Yaw]只使用磁力计解算 [E]*/
/*******************************************************************************************************************/
#endif


#if DEFINE_IMU_ANALYSIS_MODE == 5
/*******************************************************************************************************************/
/*[S] [仅输出Yaw]使用Madgwick融合解算 [S]-----------------------------------------------------------------------------*/
/*******************************************************************************************************************/

int16_t imu_quick_count = 0; // IMU稳定计数器
bool imu_stable = false; // IMU稳定标志

Madgwick_AHRS_StructDef madgwick_ahrs = {
    .q0 = 1.0f,
    .q1 = 0.0f,
    .q2 = 0.0f,
    .q3 = 0.0f,
    .beta = 1.2f,
    .quick_beta = 10.0f,
    .invSampleFreq = 0.005f,
    .mag_declination = 0.0f
};// Madgwick算法结构体实例

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     Madgwick AHRS初始化函数
// 参数说明     ahrs            Madgwick结构体指针
// 返回参数     void
// 备注信息     初始化Madgwick四元数与参数
//-------------------------------------------------------------------------------------------------------------------
void Madgwick_AHRS_Init(Madgwick_AHRS_StructDef *ahrs)
{
    if (ahrs == NULL)
    {
        return;
    }

    ahrs->q0 = 1.0f;
    ahrs->q1 = 0.0f;
    ahrs->q2 = 0.0f;
    ahrs->q3 = 0.0f;
    ahrs->beta = 1.2f;
    ahrs->quick_beta = 10.0f;
    ahrs->invSampleFreq = 0.005f;
    ahrs->mag_declination = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     Madgwick九轴偏航角解算（单一入口）
// 参数说明     dt              计算周期（秒）
// 返回参数     float           偏航角（度）
// 备注信息     使用原始数据处理链路并执行Madgwick融合
//-------------------------------------------------------------------------------------------------------------------
static float Madgwick_AHRS_Update(float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2bx, _2bz, _4bx, _4bz;
    float _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
    float q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float yaw_deg;
    float beta;

    float gx, gy, gz;
    float ax, ay, az;
    int16_t mx, my, mz;
    float q0 = madgwick_ahrs.q0;
    float q1 = madgwick_ahrs.q1;
    float q2 = madgwick_ahrs.q2;
    float q3 = madgwick_ahrs.q3;

    IMU_Acc_Apply(&ax, &ay, &az);
    IMU_Gyro_Apply(&gyro_cal, &gx, &gy, &gz);
    IMU_Mag_Apply(&mag_cal, &mx, &my, &mz);

    if (dt > 1e-6f)
    {
        madgwick_ahrs.invSampleFreq = dt;
    }

    // imu_stable=0 使用快速参数，imu_stable=1 使用正常参数
    beta = imu_stable ? madgwick_ahrs.beta : madgwick_ahrs.quick_beta;

    {
        float declination_rad = madgwick_ahrs.mag_declination * 3.1415926535f / 180.0f;
        float mx_raw = mx;
        float my_raw = my;
        mx = mx_raw * cosf(declination_rad) - my_raw * sinf(declination_rad);
        my = mx_raw * sinf(declination_rad) + my_raw * cosf(declination_rad);
    }

    recipNorm = fast_sqrt(ax * ax + ay * ay + az * az);
    if (recipNorm < 1e-6f)
    {
        return atan2f(2.0f * (q1 * q2 + q0 * q3),
                      q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180.0f / 3.1415926535f;
    }
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    recipNorm = fast_sqrt(mx * mx + my * my + mz * mz);
    if (recipNorm < 1e-6f)
    {
        return atan2f(2.0f * (q1 * q2 + q0 * q3),
                      q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180.0f / 3.1415926535f;
    }
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0q2;
    _2q2q3 = 2.0f * q2q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax)
       + _2q1 * (2.0f * q0q1 + _2q2q3 - ay)
       - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
       + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
       + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax)
       + _2q0 * (2.0f * q0q1 + _2q2q3 - ay)
       - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
       + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
       + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
       + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax)
       + _2q3 * (2.0f * q0q1 + _2q2q3 - ay)
       - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
       + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
       + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
       + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax)
       + _2q2 * (2.0f * q0q1 + _2q2q3 - ay)
       + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
       + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
       + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    recipNorm = fast_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (recipNorm >= 1e-6f)
    {
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
    }

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy) - beta * s1;
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx) - beta * s2;
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx) - beta * s3;

    q0 += qDot1 * madgwick_ahrs.invSampleFreq;
    q1 += qDot2 * madgwick_ahrs.invSampleFreq;
    q2 += qDot3 * madgwick_ahrs.invSampleFreq;
    q3 += qDot4 * madgwick_ahrs.invSampleFreq;

    recipNorm = fast_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    madgwick_ahrs.q0 = q0;
    madgwick_ahrs.q1 = q1;
    madgwick_ahrs.q2 = q2;
    madgwick_ahrs.q3 = q3;

    yaw_deg = atan2f(2.0f * (q1 * q2 + q0 * q3),
                     q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180.0f / 3.1415926535f;

    return yaw_deg;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     Madgwick AHRS 算法重置函数
// 参数说明     ahrs            Madgwick算法结构体指针
// 返回参数     void
// 使用示例     Madgwick_AHRS_Reset(&madgwick_ahrs);
// 备注信息     将四元数重置为单位四元数并重置通用状态
//-------------------------------------------------------------------------------------------------------------------
void Madgwick_AHRS_Reset(Madgwick_AHRS_StructDef *ahrs)
{
    // 四元数重置为单位四元数
    ahrs->q0 = 1.0f;
    ahrs->q1 = 0.0f;
    ahrs->q2 = 0.0f;
    ahrs->q3 = 0.0f;

    // 通用状态重置
    imu_stable = false;           // 重置稳定标志
    imu_quick_count = 0;          // 重置稳定计数器
    Yaw_Result = 0.0f;            // 重置Yaw结果输出
}

/*******************************************************************************************************************/
/*-----------------------------------------------------------------------------[E] [仅输出Yaw]使用Madgwick融合解算 [E]*/
/*******************************************************************************************************************/
#endif


#if DEFINE_IMU_ANALYSIS_MODE == 6
/*******************************************************************************************************************/
/*[S] [仅输出Yaw]使用重力投影磁修正陀螺积分 [S]-------------------------------------------------------------------------*/
/*******************************************************************************************************************/

int16_t imu_quick_count = 0; // IMU稳定计数器
bool imu_stable = false; // IMU稳定标志

Mag_Get_Yaw_StructDef Mag_ahrs = {
    .mag_declination = -98.0f,
    .yaw_filter_alpha = 0.3f,
    .yaw_filtered = 0.0f
};// 磁力计获取偏航角结构体实例

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     角度归一化到[-180, 180]
// 参数说明     angle           输入角度（度）
// 返回参数     float           归一化后的角度（度）
//-------------------------------------------------------------------------------------------------------------------
static float wrap_angle_deg(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     三轴yaw参数初始化函数
// 参数说明     imu             三轴参数结构体指针
// 返回参数     void
// 使用示例     Mag_Get_Yaw_StructDef imu3axis; Mag_Get_Yaw_Init(&imu3axis);
// 备注信息     初始化磁偏角与yaw滤波参数
//-------------------------------------------------------------------------------------------------------------------
void Mag_Get_Yaw_Init(Mag_Get_Yaw_StructDef *imu)
{
    imu->mag_declination = -98.0f;
    imu->yaw_filter_alpha = 0.3f;
    imu->yaw_filtered = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     三轴偏航角解算（单一入口）
// 参数说明     void
// 返回参数     float           偏航角（度）
// 备注信息     仅使用磁力计数据计算偏航角
//-------------------------------------------------------------------------------------------------------------------
static float Mag_Get_Yaw_Update(void)
{
    // 读取加速度计数据用于倾斜补偿
    float ax;
    float ay;
    float az;
    // 读取磁力计数据
    int16_t mx;
    int16_t my;
    int16_t mz;

    IMU_Acc_Apply(&ax, &ay, &az);
    IMU_Mag_Apply(&mag_cal, &mx, &my, &mz);

    // 加速度计数据归一化
    float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (acc_norm < 1e-6f) return Mag_ahrs.yaw_filtered;
    ax /= acc_norm;
    ay /= acc_norm;
    az /= acc_norm;

    // 磁力计数据归一化
    float mag_norm = sqrtf(mx * mx + my * my + mz * mz);
    if (mag_norm < 1e-6f) return Mag_ahrs.yaw_filtered;
    mx /= mag_norm;
    my /= mag_norm;
    mz /= mag_norm;

    // 使用加速度构造仅含横滚/俯仰的四元数，将磁力计旋转到水平坐标系
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    float cr = cosf(0.5f * roll);
    float sr = sinf(0.5f * roll);
    float cp = cosf(0.5f * pitch);
    float sp = sinf(0.5f * pitch);

    // yaw=0 的 ZYX 欧拉角转四元数（机体系 -> 水平系）
    float q0 = cr * cp;
    float q1 = sr * cp;
    float q2 = cr * sp;
    float q3 = -sr * sp;

    // 旋转矩阵第一、二行即可得到水平面分量
    float r11 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    float r12 = 2.0f * (q1 * q2 - q0 * q3);
    float r13 = 2.0f * (q1 * q3 + q0 * q2);
    float r21 = 2.0f * (q1 * q2 + q0 * q3);
    float r22 = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
    float r23 = 2.0f * (q2 * q3 - q0 * q1);

    float mx_h = r11 * mx + r12 * my + r13 * mz;
    float my_h = r21 * mx + r22 * my + r23 * mz;

    // 检查水平分量有效性
    float mag_h_norm = sqrtf(mx_h * mx_h + my_h * my_h);
    if (mag_h_norm < 1e-6f) return Mag_ahrs.yaw_filtered;

    // 水平分量归一化
    mx_h /= mag_h_norm;
    my_h /= mag_h_norm;

    // 计算偏航角
    float yaw_rad = atan2f(my_h, mx_h);
    float yaw_deg = yaw_rad * 180.0f / 3.1415926535f;

    // 应用磁偏角
    yaw_deg += Mag_ahrs.mag_declination;

    // 归一化到 -180 到 180 度范围
    float yaw_mag = -yaw_deg;
    while (yaw_mag > 180.0f) yaw_mag -= 360.0f;
    while (yaw_mag < -180.0f) yaw_mag += 360.0f;
    
    return -yaw_mag;
}

TiltMagYaw_StructDef tilt_mag_yaw_ahrs;// 重力投影磁修正陀螺积分结构体实例

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     重力投影磁修正陀螺积分初始化函数
// 参数说明     ahrs            新算法结构体指针
// 返回参数     void
// 备注信息     初始化Yaw积分状态、修正增益和输出滤波参数
//-------------------------------------------------------------------------------------------------------------------
void TiltMagYaw_Init(TiltMagYaw_StructDef *ahrs)
{
    if (ahrs == NULL)
    {
        return;
    }

    ahrs->yaw = 0.0f;
    ahrs->yaw_filtered = 0.0f;
    ahrs->yaw_error_int = 0.0f;
    ahrs->kp = 0.08f;
    ahrs->ki = 0.0025f;
    ahrs->quick_kp = 0.03f;
    ahrs->quick_ki = 0.0010f;
    ahrs->yaw_filter_alpha = 0.3f;
    ahrs->mag_declination = 270.0f;
}


//-------------------------------------------------------------------------------------------------------------------
//  函数简介     重力投影磁修正陀螺积分Yaw解算
// 参数说明     dt              计算周期（秒）
// 返回参数     float           偏航角（度）
// 备注信息     先用加速度估计重力方向，再将磁力计投影到水平面，最后用磁航向纠正陀螺积分
//-------------------------------------------------------------------------------------------------------------------
static float TiltMagYaw_Update(float dt)
{
    static uint8_t yaw_initialized = 0;
    static float q_gyro[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // 四元数状态
    static float yaw_offset = 0.0f; // 将陀螺积分yaw映射到北向参考的偏置

    float gx, gy, gz;
    float ax, ay, az;
    
    // 时间戳检查
    if (dt <= 1e-6f) dt = 0.005f;

    IMU_Acc_Apply(&ax, &ay, &az);
    IMU_Gyro_Apply(&gyro_cal, &gx, &gy, &gz);

    // 仅用于评估加速度计质量，避免剧烈机动时过度依赖磁力计
    float acc_norm_raw = sqrtf(ax * ax + ay * ay + az * az);
    if (acc_norm_raw < 1e-6f) acc_norm_raw = 1.0f;

    // 使用与 imu_transform_gyro 同源的四元数积分，得到动态yaw
    float q0 = q_gyro[0], q1 = q_gyro[1], q2 = q_gyro[2], q3 = q_gyro[3];
    q_gyro[0] += (-q1 * gx - q2 * gy - q3 * gz) * 0.5f * dt;
    q_gyro[1] += ( q0 * gx + q2 * gz - q3 * gy) * 0.5f * dt;
    q_gyro[2] += ( q0 * gy - q1 * gz + q3 * gx) * 0.5f * dt;
    q_gyro[3] += ( q0 * gz + q1 * gy - q2 * gx) * 0.5f * dt;
    
    // 四元数归一化
    float norm = sqrtf(q_gyro[0] * q_gyro[0] + q_gyro[1] * q_gyro[1] + q_gyro[2] * q_gyro[2] + q_gyro[3] * q_gyro[3]);
    if (norm > 1e-6f)
    {
        norm = 1.0f / norm;
        q_gyro[0] *= norm;
        q_gyro[1] *= norm;
        q_gyro[2] *= norm;
        q_gyro[3] *= norm;
    }

    // 从四元数计算陀螺仪偏航角
    float yaw_gyro = atan2f(2.0f * q_gyro[1] * q_gyro[2] + 2.0f * q_gyro[0] * q_gyro[3], 
                          -2.0f * q_gyro[2] * q_gyro[2] - 2.0f * q_gyro[3] * q_gyro[3] + 1.0f) * 57.2957f;
    yaw_gyro = wrap_angle_deg(yaw_gyro);

    // 绝对北向参考：磁力计yaw
    float yaw_mag = Mag_Get_Yaw_Update();

    // 快速磁力计可用性检查，仅用于决定是否进行磁修正
    float mx_raw = imu963ra_mag_x;
    float my_raw = imu963ra_mag_y;
    float mz_raw = imu963ra_mag_z;
    float mag_norm_raw = sqrtf(mx_raw * mx_raw + my_raw * my_raw + mz_raw * mz_raw);
    uint8_t mag_valid = (mag_norm_raw > 1e-6f) ? 1 : 0;

    // 初始化处理
    if (!yaw_initialized)
    {
        float yaw_init = yaw_gyro;
        if (mag_valid)
        {
            yaw_offset = wrap_angle_deg(yaw_mag - yaw_gyro);
            yaw_init = yaw_mag;
        }

        tilt_mag_yaw_ahrs.yaw = yaw_init;
        tilt_mag_yaw_ahrs.yaw_filtered = yaw_init;
        tilt_mag_yaw_ahrs.yaw_error_int = 0.0f;
        yaw_initialized = 1;
        Yaw_Result = yaw_init;
        return yaw_init;
    }

    // 先用陀螺积分预测，再叠加北向偏置
    float yaw_pred = wrap_angle_deg(yaw_gyro + yaw_offset);
    float yaw = yaw_pred;

    if (mag_valid)
    {
        // 动态调整PI参数
        float kp = imu_stable ? tilt_mag_yaw_ahrs.kp : tilt_mag_yaw_ahrs.quick_kp;
        float ki = imu_stable ? tilt_mag_yaw_ahrs.ki : tilt_mag_yaw_ahrs.quick_ki;

        float acc_quality = 1.0f - fabsf(acc_norm_raw - 1.0f) * 2.0f;
        acc_quality = acc_quality < 0.0f ? 0.0f : (acc_quality > 1.0f ? 1.0f : acc_quality);
        kp *= (0.30f + 0.70f * acc_quality);
        ki *= (0.20f + 0.80f * acc_quality);

        // 基于预测值与磁北绝对值的误差做修正
        float yaw_error = wrap_angle_deg(yaw_mag - yaw_pred);
        tilt_mag_yaw_ahrs.yaw_error_int += ki * yaw_error * dt;

        if (tilt_mag_yaw_ahrs.yaw_error_int > 25.0f) tilt_mag_yaw_ahrs.yaw_error_int = 25.0f;
        else if (tilt_mag_yaw_ahrs.yaw_error_int < -25.0f) tilt_mag_yaw_ahrs.yaw_error_int = -25.0f;

        yaw = wrap_angle_deg(yaw_pred + kp * yaw_error + tilt_mag_yaw_ahrs.yaw_error_int);
    }
    else
    {
        // 磁力计失效时保持运动连续，抑制积分项继续漂移
        tilt_mag_yaw_ahrs.yaw_error_int *= 0.98f;
    }

    // 实时更新偏置：既保持北向锚定，又保留陀螺动态属性
    yaw_offset = wrap_angle_deg(yaw - yaw_gyro);

    // 低通滤波
    float delta = wrap_angle_deg(yaw - tilt_mag_yaw_ahrs.yaw_filtered);
    tilt_mag_yaw_ahrs.yaw_filtered = wrap_angle_deg(tilt_mag_yaw_ahrs.yaw_filtered + tilt_mag_yaw_ahrs.yaw_filter_alpha * delta);

    // 更新状态
    tilt_mag_yaw_ahrs.yaw = yaw;
    Yaw_Result = tilt_mag_yaw_ahrs.yaw_filtered;

    return Yaw_Result;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     重力投影磁修正陀螺积分重置函数
// 参数说明     ahrs            TiltMagYaw算法结构体指针
// 返回参数     void
// 使用示例     TiltMagYaw_Reset(&tilt_mag_yaw_ahrs);
// 备注信息     重置Yaw角度、积分项和通用状态
//-------------------------------------------------------------------------------------------------------------------
void TiltMagYaw_Reset(TiltMagYaw_StructDef *ahrs)
{
    // Yaw角度重置
    ahrs->yaw = 0.0f;
    ahrs->yaw_filtered = 0.0f;
    
    // 误差积分项清零
    ahrs->yaw_error_int = 0.0f;

    // 通用状态重置
    imu_stable = false;           // 重置稳定标志
    imu_quick_count = 0;          // 重置稳定计数器
    Yaw_Result = 0.0f;            // 重置Yaw结果输出
}

/*******************************************************************************************************************/
/*-------------------------------------------------------------------------[E] [仅输出Yaw]使用重力投影磁修正陀螺积分 [E]*/
/*******************************************************************************************************************/
#endif






//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU姿态解算调用函数
// 备注信息     在定时器中断中调用，根据DEFINE_IMU_ANALYSIS_MODE选择解算模式
//-------------------------------------------------------------------------------------------------------------------
void IMU_Update_Analysis(void)
{
    IMU_Update_Data();                  // 读原始数据

    // 0 关闭
    // 1 三轴
    // 2 六轴
    // 3 九轴
    // 4 [仅输出Yaw]Mag_Get_Yaw(仅磁力计+倾斜补偿)
    // 5 [仅输出Yaw]Mahony AHRS(九轴)
    // 6 [仅输出Yaw]Madgwick AHRS(九轴)
    // 7 [仅输出Yaw]TiltMagYaw(重力投影磁修正陀螺积分)
    
    #if   DEFINE_IMU_ANALYSIS_MODE == 0 
        Yaw_Result   = 0.0f;
        Roll_Result  = 0.0f;
        Pitch_Result = 0.0f;
    #elif DEFINE_IMU_ANALYSIS_MODE == 1// [仅输出Yaw]三轴
        imu_transform_gyro();
    #elif DEFINE_IMU_ANALYSIS_MODE == 2// [仅输出Yaw]六轴
        IMU_getEulerianAngles();
    // 使用了磁力计的四个方法
    #elif (DEFINE_IMU_ANALYSIS_MODE == 3) || \
          (DEFINE_IMU_ANALYSIS_MODE == 4) || \
          (DEFINE_IMU_ANALYSIS_MODE == 5) || \
          (DEFINE_IMU_ANALYSIS_MODE == 6)

        // 等待系统稳定
        // 初始化阶段使用快速收敛参数，稳定后使用正常参数
        if(imu_stable == false)
        {
            if(imu_quick_count < 200)
            {
                imu_quick_count++;
            }    
            else
            {
                imu_stable = true;
            }
        }
        if (IMU_Mag_Calib_Check(&mag_cal) == MAG_CALIB_STATE_DONE)
        {
            #if DEFINE_IMU_ANALYSIS_MODE == 3 // [仅输出Yaw]九轴融合解算
                Yaw_Result = Mahony_AHRS_Update(0.01);
            #elif DEFINE_IMU_ANALYSIS_MODE == 4 // [仅输出Yaw]只使用磁力计解算
                Yaw_Result = Mag_Get_Yaw_Update();
            #elif DEFINE_IMU_ANALYSIS_MODE == 5 // [仅输出Yaw]使用Madgwick融合解算
                Yaw_Result = Madgwick_AHRS_Update(0.01);
            #elif DEFINE_IMU_ANALYSIS_MODE == 6 // [仅输出Yaw]使用重力投影磁修正陀螺积分
                Yaw_Result = TiltMagYaw_Update(0.01);
            #endif
        }
    #endif
}
/*======================================================*/
/**********************************************[姿态解算]*/
/*======================================================*/






/*======================================================*/
/*[外部调用工具性函数]**************************************/
/*======================================================*/

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置IMU姿态角数据
// 备注信息     调试性质
//-------------------------------------------------------------------------------------------------------------------
void IMU_Reset_Data (void)
{
    // 重置对应算法的中间数据
    // 0 关闭
    // 1 三轴
    // 2 六轴
    // 3 九轴
    // 4 [仅输出Yaw]Mag_Get_Yaw(仅磁力计+倾斜补偿)
    // 5 [仅输出Yaw]Mahony AHRS(九轴)
    // 6 [仅输出Yaw]Madgwick AHRS(九轴)
    // 7 [仅输出Yaw]TiltMagYaw(重力投影磁修正陀螺积分)


    #if   DEFINE_IMU_ANALYSIS_MODE == 0 
        Yaw_Result = 0.0f;
    #elif DEFINE_IMU_ANALYSIS_MODE == 1 // [仅输出Yaw]三轴
        imu_zero_yaw();
    #elif DEFINE_IMU_ANALYSIS_MODE == 2 // [仅输出Yaw]六轴
        ICM_zero_yaw();
    #elif DEFINE_IMU_ANALYSIS_MODE == 3 // [仅输出Yaw]九轴融合解算
        Mahony_AHRS_Reset(&Mahony_ahrs);
    #elif DEFINE_IMU_ANALYSIS_MODE == 4 // [仅输出Yaw]只使用磁力计解算
        Mag_Get_Yaw_Reset(&Mag_ahrs);
    #elif DEFINE_IMU_ANALYSIS_MODE == 5 // [仅输出Yaw]使用Madgwick融合解算
        Madgwick_AHRS_Reset(&madgwick_ahrs);
    #elif DEFINE_IMU_ANALYSIS_MODE == 6 // [仅输出Yaw]使用重力投影磁修正陀螺积分
        TiltMagYaw_Reset(&tilt_mag_yaw_ahrs);
    #endif
}
/*======================================================*/
/**************************************[外部调用工具性函数]*/
/*======================================================*/




