#include "location.h"
#include "Encoder.h"
#include "HWT101.h"
#include "math.h"
#include "PID_COUNTER.h"
#include "Motor.h"
#include "MY_USART.h"
extern float X_real, Y_real;
  int lookforsa2 = 0;
#define pi 3.14159f
  LOCATION location;


void X_AND_Y_Init(void) // 初始化 yaw 角度
{
    location.x = 0;
    location.y = 0;
	location.x_record=0;
	location.x_relative=0;
	location.y_record=0;
	location.y_relative=0;
}

extern HWT101_USART HWT101;
float sa_angle = 0.0f;
float sa103, sa104;
// 获取 XY 坐标，通过偏航角和速度计算当前位置，每 5ms 更新一次
void X_AND_Y_GET(float *x, float *y)
{
    float sa1_real, sa2_real;
   
    sa_angle = pi * HWT101.angle / 180.0f;

    READ_SPEED();

    sa1_real = location.speed_l_now * 6.5f * pi / 4680.0f;
    sa2_real =  location.speed_R_now * 6.5f * pi / 4680.0f;

    location.y += (cos(sa_angle) * sa1_real + cos(sa_angle) * sa2_real) / 2.0f;
    location.x += (-sin(sa_angle) * sa1_real - sin(sa_angle) * sa2_real) / 2.0f;

	location.y_relative= location.y-location.y_record;//得到相对值，开局记录值为0，也就是无所谓
	location.x_relative= location.x-location.x_record;
	
    *x = (float)location.x;
    *y = (float)location.y;
}

int sa_location = 0;
float currentX = 0.0f, currentY = 0.0f;
float change_x = 0.0f, change_y = 0.0f; // 偏移量
float circle_middle_x = 0.0f, circle_middle_y = 0.0f;
float radius = 0.0f; // 半径
float L = 0.0f;
float alpha = 0.0f, BETA = 0.0f, yaw_xita = 0.0f;
int sa_center_1 = 0.0f, sa_center_2 = 0.0f;
float x1_center = 0.0f;
float x2_center = 0.0f;
float y1_center = 0.0f;
float y2_center = 0.0f;
int CURRENT_x = 0.0f;
int CURRENT_y = 0.0f;
// 当前坐标系的 xy，表示当前位置的 xy
int location_yaw = 0;
float k_AB = 0;
float k_middle = 0;
float c_middle = 0;
float deta = 0;
float distance = 0;
float deta_dx = 0, deta_dy = 0;
float deta_a = 0;
float deta_b = 0;
float deta_c = 0;
float deta_staue = 0;
float x_1 = 0, x_2 = 0;
float y_1 = 0, y_2 = 0;

void location_GO(int expect_x, int expect_y, float current_x, float current_y) // 实时路径规划
{
    if (sa_location == 0) // 第一次运行时获取初始位置
    {
        location_yaw = 5.2f;
        currentX = (current_x);
        currentY = (current_y);
        sa_location = 1;
    }

    L = sqrt((currentX - expect_x) * (currentX - expect_x) + (currentY - expect_y) * (currentY - expect_y));

    alpha = atan2((currentX - expect_x), (currentY - expect_y)) * 180.0f / pi;
    BETA = 90.0f - alpha - location_yaw;
    BETA = BETA * (pi / 180);

    radius = ((L / 2.0f) / cos(BETA)); // 计算半径

    distance = sqrt(radius * radius - (L / 2.0f) * (L / 2.0f));
    float x_m = (expect_x + currentX) / 2.0f;
    float y_m = (expect_y + currentY) / 2.0f;
    k_AB = (expect_y - currentY) / (expect_x - currentX); // 斜率
    k_middle = (-1) / k_AB;
    c_middle = y_m - k_middle * x_m;

    deta_a = (1 + k_middle * k_middle);
    deta_b = 2 * (-x_m + k_middle * c_middle - y_m * k_middle);
    deta_c = x_m * x_m + c_middle * c_middle + y_m * y_m - 2 * y_m * c_middle - distance * distance;

    deta = deta_b * deta_b - 4 * deta_a * deta_c;

    if (deta < 0)
    {
        deta_staue = 1;
    }
    if (deta >= 0)
    {
        deta_staue = 2;
        x_1 = (-deta_b + sqrt(deta)) / (2 * deta_a);
        y_1 = k_middle * x_1 + c_middle;

        x_2 = (-deta_b - sqrt(deta)) / (2 * deta_a);
        y_2 = k_middle * x_2 + c_middle;

        circle_middle_x = x_1;
        circle_middle_y = y_1;

        change_x = current_x - circle_middle_x;
        change_y = current_y - circle_middle_y; // 偏移量
    }
}

void Location_Go_wang(float goal_x, float goal_y, float *Distance, float *Angle_Diff) // goal_x, goal_y 为目标值，Distance, Angle_Diff 为 PID 输入
{
    float x_diff, y_diff;
    x_diff = goal_x - X_real;
    y_diff = goal_y - Y_real;
    *Distance = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
    *Angle_Diff = atan(x_diff / y_diff);
}

int sa_20 = 0, sa_21 = 0;
float sa_dx, sa_dy;
float sa_aerfa;
float sa_yaw4;
float sa_L;

void Location_GO_EYSE(float expect_x, float expect_y, float current_x, float current_y) // 目标位置
{
    sa_dx = current_x - expect_x;
    sa_dy = current_y - expect_y;

    sa_aerfa = atan2(sa_dx, sa_dy) * 180.0f / pi;

    sa_yaw4 = -180.0f - sa_aerfa;

    if (sa_yaw4 < -180)
    {
        sa_yaw4 += 360;
    }
    sa_L = sa_dx * sa_dx + sa_dy * sa_dy; // 平方

    if (sa_L >= 5)
    {
        straight_line(4000 + sa_L / 10, 4000 + sa_L / 10, sa_yaw4, &sa_20, &sa_21);
    }

    if (sa_L < 5)
    {
        Motor_370_respectively(0, 0);
    }
}

// 小车避障
int Location_stuas1 = 0;
float sa_C_x = 0, sa_C_y = 0;

void Location_strike(void)
{
    static float sa_A_x = 0, sa_B_x = 0, sa_A_yaw = 0;
    static float sa_A_y = 0, sa_B_y = 0;
    static int sa_AB_stuas = 0;
    static float current_y_bias = 0;
    static float current_y_bias_square = 0;

    if (sa_AB_stuas == 0)
    {
        sa_A_x = X_real;
        sa_A_y = Y_real;
        sa_AB_stuas = 1;
        sa_A_yaw = HWT101.angle;
    } // 获取 A 点

    static int sa_time_location = 0;
    sa_time_location++;
    // 生成 B 点

    sa_A_yaw = (sa_A_yaw * pi) / 180.0f;
    sa_B_x = sa_A_x + 20 * cos(sa_A_yaw);
    sa_B_y = sa_A_y + 20 * sin(sa_A_yaw);

    if (sa_time_location <= 100 && Location_stuas1 == 0)
    {
        current_y_bias = (Y_real - sa_A_y) * (Y_real - sa_A_y) + (X_real - sa_A_x) * (X_real - sa_A_x);
        current_y_bias_square = current_y_bias * current_y_bias;
        Motor_370_advance(7000);
    }
    // 前进
    else if (sa_time_location > 100 && sa_time_location <= 500)
    {
        Motor_370_advance(-7000);
    }
    else if (sa_time_location > 500 && sa_time_location <= 1000 && current_y_bias_square > 400)
    {
        Location_GO_EYSE(sa_B_x, sa_B_y, X_real, Y_real);
    }
    else if (sa_time_location > 1000 && sa_time_location <= 1500)
    {
        Location_GO_EYSE(sa_A_x, sa_A_y, X_real, Y_real);
    }
    else if (sa_time_location > 1500 && sa_time_location <= 1600)
    {
        direction_PID(7000, 7000, sa_A_yaw, HWT101.angle);
    }
    else if (sa_time_location > 1600)
    {
        Motor_Location_turn(4000);
    }
}

int lookforsa = 0;

int Location_turn(void)
{
    static float goto_yaw_staus = 0; // 记录是否完成一圈
    static float goto_yaw_staus_0 = 0, goto_yaw_staus_45 = 0, goto_yaw_staus_90 = 0, goto_yaw_staus_160 = 0, goto_yaw_staus_f160 = 0, goto_yaw_staus_f90 = 0, goto_yaw_staus_f45 = 0; // 记录是否完成特定角度

    if (HWT101.angle <= 10.0f && HWT101.angle >= -10.0f && goto_yaw_staus_0 == 0)
    {
        goto_yaw_staus++;
        goto_yaw_staus_0 = 1;
    }
    if (HWT101.angle <= 55.0f && HWT101.angle >= 35.0f && goto_yaw_staus_45 == 0)
    {
        goto_yaw_staus++;
        goto_yaw_staus_45 = 1;
    }
    if (HWT101.angle <= 100.0f && HWT101.angle >= 80.0f && goto_yaw_staus_90 == 0)
    {
        goto_yaw_staus++;
        goto_yaw_staus_90 = 1;
    }
    if (HWT101.angle <= 170.0f && HWT101.angle >= 150.0f && goto_yaw_staus_160 == 0)
    {
        goto_yaw_staus++;
        goto_yaw_staus_160 = 1;
    }
    if (HWT101.angle <= -150.0f && HWT101.angle >= -170.0f && goto_yaw_staus_f160 == 0)
    {
        goto_yaw_staus++;
        goto_yaw_staus_f160 = 1;
    }
    if (HWT101.angle <= -80.0f && HWT101.angle >= -100.0f && goto_yaw_staus_f90 == 0)
    {
        goto_yaw_staus++;
        goto_yaw_staus_f90 = 1;
    }
    if (HWT101.angle <= -40.0f && HWT101.angle >= -50.0f && goto_yaw_staus_f45 == 0)
    {
        goto_yaw_staus++;
        goto_yaw_staus_f45 = 1;
    }
    if (goto_yaw_staus < 7) // 360度分为8个状态
    {
        Motor_Location_turn(4000);
        return 0;
    }
    else
    {
        lookforsa++;
        goto_yaw_staus = 0;
        goto_yaw_staus_0 = 0, goto_yaw_staus_45 = 0, goto_yaw_staus_90 = 0, goto_yaw_staus_160 = 0, goto_yaw_staus_f160 = 0, goto_yaw_staus_f90 = 0, goto_yaw_staus_f45 = 0;
        return 1;
    }
}

int expect_yaw_location = 0;
int gohome_satus = 0;
int lookforwangsa = 0;
int location_pd_time = 0;
int  location_turn_common=0;
int sa_common_time=0;
 int turn_commot_time=0;
int Location_turn_PID(int lookfor, int start_yaw, int ending_yaw) // 根据当前状态获取目标角度
{
     
	if(start_yaw<=ending_yaw)
	{
    location_pd_time++;
    expect_yaw_location = location_pd_time / 20.0f + start_yaw; }
	else
	{
    location_pd_time++;
    expect_yaw_location = -location_pd_time / 20.0f + start_yaw; }
    
	if( ((expect_yaw_location - ending_yaw)*(expect_yaw_location - ending_yaw))>=4.0f)
    {                                                            
        direction_PID(5000, 5000, (float)expect_yaw_location, HWT101.angle);
        return 0;}
    else 
    {
        if (lookfor == 1)
        {
            lookforsa++;
        }
        if (lookfor == 0)
        {
            gohome_satus++;
        }
        if (lookfor == 2)
        {
            lookforwangsa++;
			
        }
		if(lookfor==3)
		{
		
			   lookforsa2++;
		
		}
		if(lookfor==4)
		{
			sa_common_time=0;
			location_turn_common++;
			location_pd_time=0;
			turn_commot_time=0;
			
		}
       
        return 1;
    }
	
}

void Location_Lookfor(void)
{
    static int car_y = 4.0f; // 小车 y 方向偏移
    static float target_difference = 0.0f;

    if (lookforsa == 0) // 初始状态
    {
        Location_GO_EYSE(0, 120.0f - car_y, X_real, Y_real);
        target_difference = (0.0f - X_real) * (0.0f - X_real) + ((120.0f - car_y) - Y_real) * ((120.0f - car_y) - Y_real);
        if (target_difference <= 10)
        {
            lookforsa++;
        }
    }
    if (lookforsa == 1) // 原地转向
    {
        while (Location_turn_PID(1, -175, 175) == 0)
        {
            return;
        }
    }
    if (lookforsa == 2)
    {
        Location_GO_EYSE(0, 150.0f - car_y, X_real, Y_real); // 目标位置
        target_difference = (0.0f - X_real) * (0.0f - X_real) + ((150.0f - car_y) - Y_real) * ((150.0f - car_y) - Y_real);
        if (target_difference <= 10)
        {
            lookforsa++;
        }
    }
    if (lookforsa == 3)
    {
        while (Location_turn_PID(1, -175, 175) == 0)
        {
            return;
        }
    }
    if (lookforsa == 4)
    {
        Location_GO_EYSE(-30, 150.0f - car_y, X_real, Y_real);
        target_difference = (-30.0f - X_real) * (-30.0f - X_real) + ((150.0f - car_y) - Y_real) * ((150.0f - car_y) - Y_real);
        if (target_difference <= 10)
        {
            lookforsa++;
        }
    }
    if (lookforsa == 5)
    {
        while (Location_turn_PID(1, -175, 175) == 0)
        {
            return;
        }
    }
    if (lookforsa == 6)
    {
        Location_GO_EYSE(-20, 120.0f - car_y, X_real, Y_real);
        target_difference = (-20.0f - X_real) * (-20.0f - X_real) + ((120.0f - car_y) - Y_real) * ((120.0f - car_y) - Y_real);
        if (target_difference <= 10)
        {
            lookforsa++;
        }
    }
    if (lookforsa == 7)
    {
        while (Location_turn_PID(1, -175, 175) == 0)
        {
            return;
        }
    }
    if (lookforsa == 8)
    {
        Location_GO_EYSE(-30, 90.0f - car_y, X_real, Y_real);
        target_difference = (-30.0f - X_real) * (-30.0f - X_real) + ((90.0f - car_y) - Y_real) * ((90.0f - car_y) - Y_real);
        if (target_difference <= 10)
        {
            lookforsa++;
        }
    }
    if (lookforsa == 9)
    {
        while (Location_turn_PID(1, -175, 175) == 0)
        {
            return;
        }
    }
    if (lookforsa == 10)
    {
        Location_GO_EYSE(30, 90.0f - car_y, X_real, Y_real);
        target_difference = (30.0f - X_real) * (30.0f - X_real) + ((90.0f - car_y) - Y_real) * ((90.0f - car_y) - Y_real);
        if (target_difference <= 10)
        {
            lookforsa++;
        }
    }
    if (lookforsa == 11)
    {
        while (Location_turn_PID(1, -175, 175) == 0)
        {
            return;
        }
    }
    if (lookforsa == 12)
    {
        Location_GO_EYSE(30, 135.0f - car_y, X_real, Y_real);
        target_difference = (30.0f - X_real) * (30.0f - X_real) + ((135.0f - car_y) - Y_real) * ((135.0f - car_y) - Y_real);
        if (target_difference <= 10)
        {
            lookforsa++;
        }
    }
    if (lookforsa == 13)
    {
        while (Location_turn_PID(1, -175, 175) == 0)
        {
            return;
        }
    }
    if (lookforsa == 14)
    {
        Location_GO_EYSE(30, 180.0f - car_y, X_real, Y_real);
        target_difference = (30.0f - X_real) * (30.0f - X_real) + ((180.0f - car_y) - Y_real) * ((180.0f - car_y) - Y_real);
        if (target_difference <= 10)
        {
            lookforsa++;
        }
    }
    if (lookforsa == 15)
    {
        while (Location_turn_PID(1, -175, 175) == 0)
        {
            return;
        }
    }
    if (lookforsa == 16)
    {
        Location_GO_EYSE(-60, 180.0f - car_y, X_real, Y_real);
        target_difference = (-60.0f - X_real) * (-60.0f - X_real) + ((180.0f - car_y) - Y_real) * ((180.0f - car_y) - Y_real);
        if (target_difference <= 10)
        {
            lookforsa++;
        }
    }
    if (lookforsa == 17)
    {
        while (Location_turn_PID(1, -175, 175) == 0)
        {
            return;
        }
    }
    if (lookforsa == 18)
    {
        Location_GO_EYSE(-50, 120.0f - car_y, X_real, Y_real);
        target_difference = (-50.0f - X_real) * (-50.0f - X_real) + ((120.0f - car_y) - Y_real) * ((120.0f - car_y) - Y_real);
        if (target_difference <= 10)
        {
            lookforsa++;
        }
    }
    if (lookforsa == 19)
    {
        while (Location_turn_PID(1, -175, 175) == 0)
        {
            return;
        }
    }
    if (lookforsa == 20)
    {
        Location_GO_EYSE(-60, 60.0f - car_y, X_real, Y_real);
        target_difference = (-60.0f - X_real) * (-60.0f - X_real) + ((60.0f - car_y) - Y_real) * ((60.0f - car_y) - Y_real);
        if (target_difference <= 10)
        {
            lookforsa++;
        }
    }
    if (lookforsa == 21)
    {
        while (Location_turn_PID(1, -175, 175) == 0)
        {
            return;
        }
    }
    if (lookforsa == 22)
    {
        Location_GO_EYSE(0, 60.0f - car_y, X_real, Y_real);
        target_difference = (0.0f - X_real) * (0.0f - X_real) + ((60.0f - car_y) - Y_real) * ((60.0f - car_y) - Y_real);
        if (target_difference <= 10)
        {
            lookforsa++;
        }
    }
    if (lookforsa == 23)
    {
        while (Location_turn_PID(1, -175, 175) == 0)
        {
            return;
        }
    }
    if (lookforsa == 24)
    {
        Location_GO_EYSE(60, 60.0f - car_y, X_real, Y_real);
        target_difference = (60.0f - X_real) * (60.0f - X_real) + ((60.0f - car_y) - Y_real) * ((60.0f - car_y) - Y_real);
        if (target_difference <= 10)
        {
            lookforsa++;
        }
    }
}
 int baoan_sa=0; 
 int  location_turn_common_Sa=0;
int location_goto_point(float target_x, float target_y)
{
    static int car_y = 4.0f; // 小车 y 方向偏移
    static float target_difference = 0.0f;

    target_difference = (target_x-location.x_record - location.x_relative) * (target_x-location.x_record - location.x_relative) + ((target_y-location.y_record - car_y) - location.y_relative) * ((target_y-location.y_record - car_y) - location.y_relative);
    if (target_difference > 10.0f)
    {
        Location_GO_EYSE(target_x-location.x_record, target_y - car_y-location.y_record, location.x_relative, location.y_relative);
        return 0;
    }
    else
    {
		location_turn_common_Sa=1;
        lookforwangsa++;
		lookforsa2++;
		baoan_sa++;
		location_turn_common++;
        return 1;
    }
	
	
}

float wang_location_x[100] = {0.0f, 0.0f, 0.0f, 0.0f};
float wang_location_y[100] = {120.0f, 120.0f, 120.0f, 120.0f};
float wang_location_startyaw[100] = {-175.0f, 175.0f, -175.0f, 175.0f};
float wang_location_endyaw[100] = {175.0f, -175.0f, 175.0f, -175.0f};

void Location_Lookfor_wang(void)
{
    int result = lookforwangsa / 2; // 0->0, 1->0, 2->1, 3->1 自动获取
    if (lookforwangsa % 2 == 0)
    {
        while (location_goto_point(wang_location_x[result], wang_location_y[result]) == 0)
        {
			location_pd_time=0;
            return;
        }
    }
    if (lookforwangsa % 2 == 1)
    {
        while (Location_turn_PID(2, wang_location_startyaw[result], wang_location_endyaw[result]) == 0)
        {
            return;
        }
    }
}
float wang_location_baoanx[100] = {30.0f, 30.0f, 30.0f, 30.0f,  0.0f   ,-30.0f  ,-.20f ,-30.0f,-25.0f};
float wang_location_baoany[100] = {90.0f, 120.0f, 150.0f,170.0f, 170.0f ,170.0f ,150.0f, 120.0f,90.0f};

void Location_Lookfor_baoan(void)
{
  
    
        while (location_goto_point(wang_location_baoanx[baoan_sa], wang_location_baoany[baoan_sa]) == 0)
        {
			
            return;
        }
    
   
}
float g_location_x[100] = {0.0f, 60.0f, 0.0f, -10.0f};
float g_location_y[100] = {120.0f, 150.0f, 150.0f, 120.0f};
float g_location_startyaw[100] = {-175.0f, -175.0f, -175.0f, -175.0f};
float g_location_endyaw[100] = {175.0f, 175.0f, 175.0f, 175.0f};

void Location_Lookfor_g(void)
{
    int result = lookforsa2 / 2; // 0->0, 1->0, 2->1, 3->1 自动获取
    if (lookforsa2 % 2 == 0)
    {
        while (location_goto_point(g_location_x[result], g_location_y[result]) == 0)
        {
            return;
        }
    }
    if (lookforsa2 % 2 == 1)
    {
        while (Location_turn_PID(3, g_location_startyaw[result], g_location_endyaw[result]) == 0)
        {
            return;
        }
    }
}

void Location_go_home(void)
{
    static int car_y = 4.0f; // 小车 y 方向偏移
//    static float target_difference = 0.0f;
	 Location_GO_EYSE(110.0f, 120.0f - car_y, X_real, Y_real);
//    if (gohome_satus == 0)
//    {
//        Location_GO_EYSE(60.0f, 120.0f - car_y, X_real, Y_real);
//        target_difference = (60.0f - X_real) * (60.0f - X_real) + ((120.0f - car_y) - Y_real) * ((120.0f - car_y) - Y_real);
//        if (target_difference <= 10)
//        {
//            gohome_satus++;
//        }
//    }
//    if (gohome_satus == 1)
//    {
//        while (Location_turn_PID(0, -175, 175) == 0)
//        {
//            return;
//        }
//    }
//    if (gohome_satus == 2)
//    {
//        Location_GO_EYSE(65.0f, 120.0f - car_y, X_real, Y_real);
//        target_difference = (65.0f - X_real) * (65.0f - X_real) + ((120.0f - car_y) - Y_real) * ((120.0f - car_y) - Y_real);
//    }
//    if (gohome_satus == 3)
//    {
//        while (Location_turn_PID(0, -175, 175) == 0)
//        {
//            return;
//        }
//    }
}

void location_face_to_safe(float expect_x, float expect_y, float current_x, float current_y)
{
     sa_dx = current_x - expect_x;
    sa_dy = current_y - expect_y;

    sa_aerfa = atan2(sa_dx, sa_dy) * 180.0f / pi;

    sa_yaw4 = -180 - sa_aerfa;

    if (sa_yaw4 < -180)
    {
        sa_yaw4 += 360;
    }
	 direction_PID(8000, 8000, (float)sa_yaw4, HWT101.angle);
//     Location_turn_PID(3,HWT101.angle,sa_yaw4);

}
void location_relative_ball(float lay_ball_yaw)
{
	
	


	if(lay_ball_yaw<=-150.0f||lay_ball_yaw>=150.0f)//位于高上半区域
	{
	
	
	
	}
	if(lay_ball_yaw>=-30.0f&&lay_ball_yaw<=30.0f)//位于低下半区域
	{
	
	
	
	}

}

 void location_miidle_turn_common(void)//中心旋转
{
	
//	static float sa_common_yaw=0;
	//static float common_yaw_target=0;
	if(((0.0f-location.x_record - location.x_relative) * (0.0f-location.x_record - location.x_relative) + ((120.0f-location.y_record  - location.y_relative) * ((120.0f-location.y_record - location.y_relative) <10.0f)))&&location_turn_common_Sa>=1)
	{
		 if(location_turn_common%2==1)
		 {
			if(turn_commot_time==0)
			{HWT101.turn_start_yaw=HWT101.angle;
			  turn_commot_time=1;
			}
				
			 while(Location_turn_PID(4,HWT101.turn_start_yaw,175)==0)
		{   
			
		   return;
		
		}}
		if(location_turn_common%2==0)
		 {
			 if(turn_commot_time==0)
			{HWT101.turn_start_yaw=HWT101.angle;
			  turn_commot_time=1;
			}
			 
			 while(Location_turn_PID(4,HWT101.turn_start_yaw,-175)==0)
		{
		   return;}	 }
	 }
		 
			 
			 
	
	
	else if(location_turn_common_Sa==0)
	{
	   while(location_goto_point(0.0f,120.0f)==0)
	   {
		  return;
	   } 
	
	
	}
	
	   
	
	
	
	


}
