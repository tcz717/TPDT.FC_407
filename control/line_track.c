#include "control.h"
#include "camera.h"
#include "PID.h"
#include <dfs_posix.h>
#include <finsh.h>
#include "pwm_remote.h"
#include "math.h"
#include "led.h"
extern pwm_signal_t pwm;
extern PID h_v_pid;

//PID文件保存的Magic数
#define PID_SS 0xABCD
#define PID_ES 0xDCBA

#define PID_PATH "/lt.pid"

//三种巡线模式 实际上代表各自所需转弯次数
#define STRIGHT_MODE	0
#define CRUISE_MODE		4
#define THROW_MODE		2

//基础油门
#define BASIC_THROTTLE 	530
//基础高度
#define BASIC_HEIGHT 	65.0f
//起飞时间（计数器上限）
#define TAKEOFF_TIME 	RT_TICK_PER_SECOND*0.4f
//着陆时间（计数器上限）
#define LAND_TIME		RT_TICK_PER_SECOND/2
//直线减速倾角
#define LINE_STOP		7
//转弯减速倾角
#define TURN_STOP		12.0f
//转弯时偏俯仰和翻滚 组合形成过山车式的转弯效果
#define TURN_PITCH		-2.0f
#define TURN_ROLL		-1.0f
//前进俯仰角
#define GO_PITCH		-1.5f

fc_task * line_task;
fc_task * cruise_task;
fc_task * throw_task;

struct line_pid
{
    uint16_t SS;
    PID dist;
    PID angle;
    uint16_t ES;
}pid;


//使用倾角和加速度互补滤波的得到水平速度的估算值
float vx=0,vy=0;
void Ix()
{
    vx+=cosf(toRad(ahrs.degree_pitch))*ahrs.acc_y*ahrs.time_span;	
}
void Iy()
{
    vy+=sinf(toRad(ahrs.degree_roll))*ahrs.time_span;	
}

/*
*****巡线核心函数******
巡线控制代码使用协程方式编写，以便于编写时的逻辑流畅
tPre 为初始化部分
tBegin 为协程状态机开始
tReturn 用于协程迭代返回

mode为巡线模式： 直线 巡航 拾取&抛弃
*/
rt_err_t line_track(u8 mode)
{
    static float target_height; //定高目标高度
    static float current_yaw;   //目前偏航角
    static float target_yaw;    //目标倾航角
    static uint8_t wait_cnt;    //巡线丢失后重试计数器
    static int turn_cnt;        //转弯计数器（一共四次转弯）
    static int takeoff_cnt;     //起飞等待计数器
    static float left;          //左转向趋势值
    static int8_t stop;         //刹车倾角
    static int i;
    
    tPre;
    if(current_task->reset)
    {
        tReset;
        current_task->reset=RT_FALSE;
    }
    tBegin;
    PID_Reset(&pid.dist);
    PID_Reset(&pid.angle);
    target_height=0;
    stop=0;
    left=0;
    turn_cnt=0;
    vx=0;
    
    takeoff_cnt=2000;
    
    //等待起飞按钮按下
    LED4(0);
    while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0)==Bit_RESET)
    {
        LED3(1);
        tReturn(RT_EOK);
    }
    
    //记录起飞时偏航角
    current_yaw=ahrs.degree_yaw;
    LED3(0);
    //起飞倒计时
    while(takeoff_cnt>0)
    {
        LED4(takeoff_cnt/200+1);
        takeoff_cnt--;
        tReturn(RT_EOK);
    }

//起飞阶段    
//takeoff:
    rt_kprintf("takeoff.\n");
    GPIO_WriteBit(GPIOE,GPIO_Pin_1,Bit_RESET);
    GPIO_WriteBit(GPIOE,GPIO_Pin_2,Bit_SET);
    while(ahrs.height<49.0f&&target_height<95.0f)//take off
    {
        Ix();
        Iy();
        target_height=linear(target_height,5.0f,100,RT_TICK_PER_SECOND*0.7f);
        stable(-2.5f,0,current_yaw);
        
        h_v_pid.maxi=150;
        if(mode==THROW_MODE)
            h_v_pid.maxi=300;
        
        althold(BASIC_HEIGHT);
        
        motor_hupdate(450+(u16)target_height);
#ifdef OUTPUT_LINETRACK_HEIGHT        
		rt_kprintf("%d/%d\n",(u8)ahrs.height,(u8)target_height);
#endif
        
        tReturn(RT_EOK);
    }
    rt_uint32_t dump;
//直线巡线阶段
line: 
    wait_cnt=0;
    GPIO_WriteBit(GPIOE,GPIO_Pin_1,Bit_RESET);
    GPIO_WriteBit(GPIOE,GPIO_Pin_2,Bit_SET);
    while(1) //line track 
    {
        //等待摄像头信号
        if (rt_event_recv(&ahrs_event, AHRS_EVENT_CARMERA, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO, &dump) == RT_EOK)
        {
            extern int tlog(const char * fmt ,...);
            tlog("e:%d\to:%d\th:%d\tv:%d\ts:%d\n",(s16)ahrs.line_err,(s16)pid.dist.out,(s16)ahrs.height,(s16)(vx*100),(s16)recv.pack.linestate);
            
            switch(recv.pack.linestate)
            {
                //检测到交叉线
                case LINE_MARK:
                    //若在巡航模式转完圈则降落
                    if(turn_cnt>=mode&&mode==CRUISE_MODE)
                    {
                        stop=-1;
                        goto land;
                    }
                //接测到直线 巡线前进
                case LINE_STRAIGHT:
                    PID_SetTarget(&pid.dist,0);
                    PID_xUpdate(&pid.dist,ahrs.line_err);
                    left*=0.5f;
                    wait_cnt*=0.8f;
                    break;
                //检测到终止线 准备降落
                case LINE_END:
                    stop=3;
                    if(wait_cnt>=2.0f)
                    {
                        //拾取模式时抛下物体
                        if(mode==THROW_MODE)
                        {
                            stop=4;
                            if(turn_cnt==0)
                            {
                                GPIO_WriteBit(GPIOE,GPIO_Pin_2,Bit_RESET);
                                GPIO_WriteBit(GPIOE,GPIO_Pin_1,Bit_RESET);
                                goto prepare_turn;
                            }
                        }
                        if(turn_cnt>=mode)
                            goto land;
                    }
                //未检测到线 保持原方向 丢失计数器增长
                case LINE_LOST_ERROR:
                    stop=3;
                    //排除摄像头光线干扰
                    if(abs(recv.pack.middle_error)>100)
                        wait_cnt+=0.5f;
                    else
                        wait_cnt+=1.0f;
                    //长时间未检测到线 降落
                    if(wait_cnt>=20.0f)
                        goto land;
                    break;
                //检测到左转弯信号
                case LINE_TURN_LEFT_90:
                    //巡航模式下完成四个弯道角降落
                    if(turn_cnt>=mode&&mode==CRUISE_MODE)
                    {
                        stop=-1;
                        goto land;
                    }
                    //不是直线模式则转弯趋势增加
                    if(mode!=STRIGHT_MODE)
                    {
                        left+=1.0f;
                        if(left>3.0f)
                        {
                            goto prepare_turn;
                        }
                    }
                    break;
                default:
                    left*=0.9f;
                break;
            }
        }
        
        //更新水平速度
        Ix();
        Iy();
        //不同模式下负重不同 选择不同前进倾角、电机基础油门
        if(mode > 0)
        {
            if(turn_cnt<mode)
            {
                if(mode==THROW_MODE&&turn_cnt==1)
                {
                    stable(0,RangeValue(pid.dist.out,-10,10),current_yaw);
                }
                else
                    stable(GO_PITCH-GO_PITCH*turn_cnt/(mode+1),RangeValue(pid.dist.out,-10,10),current_yaw);
            }
            else
                stable(GO_PITCH/3,RangeValue(pid.dist.out,-10,10),current_yaw);
        }
        else
        {
            stable(-1.0,RangeValue(pid.dist.out,-10,10),current_yaw);
        }
        althold(BASIC_HEIGHT);
        
        if(mode==THROW_MODE&&turn_cnt==0)
            motor_hupdate(BASIC_THROTTLE+30);
        else
            motor_hupdate(BASIC_THROTTLE);
            
        
        tReturn(RT_EOK);
    }
//转弯前减速阶段
prepare_turn:
    rt_kprintf("stop at %d.\n",(u8)ahrs.height);
    extern PID h_v_pid;
    if(turn_cnt==0&&mode==THROW_MODE)
        PID_Reset(&h_v_pid);
    //转弯前减速
    for(i=0;i<80;i++)
    {
        Ix();
        Iy();
        if((turn_cnt==1||turn_cnt==3)&&mode==CRUISE_MODE)
            stable(LINE_STOP*2.0f,0,current_yaw);
        else	
            stable(LINE_STOP,0,current_yaw);
        
        if(mode==THROW_MODE)
            althold(40);
        else
            althold(BASIC_HEIGHT);
        motor_hupdate(BASIC_THROTTLE);
        GPIO_WriteBit(GPIOE,GPIO_Pin_2,Bit_RESET);
        GPIO_WriteBit(GPIOE,GPIO_Pin_1,Bit_RESET);
        tReturn(RT_EOK);
    }
//左转阶段
turn_left:
    //目标角度改变90度
    target_yaw=rangeYaw(current_yaw-90.0f);
    left=0;
    turn_cnt++;
    rt_kprintf("turn_cnt left to %d.\n",(s16)target_yaw);
    while(1)
    {
        float diff;
        diff=diffYaw(ahrs.degree_yaw,target_yaw);
        Ix();
        Iy();

        //检测到直线立马退出
        if((diff<10.0f&&diff>-10.0f)||(diff<30.0f&&diff>-30.0f&&recv.pack.linestate==LINE_STRAIGHT))
        {
            current_yaw=target_yaw;			
            vx=0;
            //抛物模式再转一圈
            if(mode==THROW_MODE&&turn_cnt==1)
                goto turn_left;
            goto line;
        }
        //不同模式下负重不同 选择不同前进倾角、电机基础油门
        if(mode==THROW_MODE)
        {
            h_v_pid.maxi=150;
            althold(40);
            if(turn_cnt==1)
                stable(2,0,target_yaw);
            else
                stable(-1.0f,-2.0f,target_yaw);
        }
        else
        {
            althold(BASIC_HEIGHT);
            stable(TURN_PITCH,RangeValue(pid.dist.out,-3,3)+TURN_ROLL,target_yaw);
        }
        motor_hupdate(BASIC_THROTTLE);
        tReturn(RT_EOK);
    }
//降落阶段
land:
    rt_kprintf("land %d.\n",(s16)stop);
    target_height=BASIC_HEIGHT;
    //在高度低于一定程度直接停下
    while(target_height>5.0f&&ahrs.height>20.0f)//land
    {
        target_height=linear(target_height,BASIC_HEIGHT,0,RT_TICK_PER_SECOND/2);
        stable(stop,0,current_yaw);
        
        motor_update(450);
        
        tReturn(RT_EOK);
    }
    GPIO_WriteBit(GPIOE,GPIO_Pin_2,Bit_RESET);
    GPIO_WriteBit(GPIOE,GPIO_Pin_1,Bit_RESET);
    tReturn(1);
    tFinish
    return 1;
}

static void init_pid()
{
    int fd;
    fd = open(PID_PATH, O_RDWR | O_CREAT, 0);

    if (fd >= 0)
    {
        if (read(fd, &pid, sizeof(pid)) != sizeof(pid) ||
            pid.SS != PID_SS || pid.ES != PID_ES)
        {
            rt_kprintf("init line track pid.\n");
            pid.SS = PID_SS;
            pid.ES = PID_ES;
            
            PID_Init(&pid.angle,0,0,0);
            PID_Init(&pid.dist,0,0,0);
            
            PID_Set_Filt_Alpha(&pid.angle,1.0f/166.0f,20.0);
            PID_Set_Filt_Alpha(&pid.dist,1.0f/60.0f,20.0);

            write(fd, &pid, sizeof(pid));
        }
        else
        {
            PID_Reset(&pid.angle);
            PID_Reset(&pid.dist);
            PID_Set_Filt_Alpha(&pid.angle,1.0f/166.0f,20.0);
            PID_Set_Filt_Alpha(&pid.dist,1.0f/60.0f,20.0);
            rt_kprintf("line track pid load succeed.\n");
        }
        close(fd);
    }
    else
    {
        rt_kprintf("line track open wrong.\n");
    }
    rt_kprintf("line angle:		%d.%03d	%d.%02d	%d.%03d.\n", (s32)pid.angle.p, (s32)(pid.angle.p*1000.0f) % 1000,
    (s32)pid.angle.i, (s32)(pid.angle.i*100.0f) % 100,
    (s32)pid.angle.d, (s32)(pid.angle.d*1000.0f) % 1000);
    rt_kprintf("line dist :		%d.%03d	%d.%02d	%d.%03d.\n", (s32)pid.dist.p, (s32)(pid.dist.p*1000.0f) % 1000,
    (s32)pid.dist.i, (s32)(pid.dist.i*100.0f) % 100,
    (s32)pid.dist.d, (s32)(pid.dist.d*1000.0f) % 1000);
}

static void save_pid()
{
    int fd;

    fd = open(PID_PATH, O_WRONLY | O_TRUNC, 0);

    if (fd >= 0)
    {
        write(fd, &pid, sizeof(pid));
        close(fd);
    }
    rt_kprintf("line angle:		%d.%03d	%d.%02d	%d.%03d.\n", (s32)pid.angle.p, (s32)(pid.angle.p*1000.0f) % 1000,
    (s32)pid.angle.i, (s32)(pid.angle.i*100.0f) % 100,
    (s32)pid.angle.d, (s32)(pid.angle.d*1000.0f) % 1000);
    rt_kprintf("line dist :		%d.%03d	%d.%02d	%d.%03d.\n", (s32)pid.dist.p, (s32)(pid.dist.p*1000.0f) % 1000,
    (s32)pid.dist.i, (s32)(pid.dist.i*100.0f) % 100,
    (s32)pid.dist.d, (s32)(pid.dist.d*1000.0f) % 1000);
}

//巡线参数远程设定
void set_la(s16 p, s16 i, s16 d)
{
    PID_Init(&pid.angle, p / 1000.0f, i / 100.0f, d / 1000.0f);
    save_pid();
}
FINSH_FUNCTION_EXPORT(set_la, set the value of pid in line track angle);
void set_ld(s16 p, s16 i, s16 d)
{
    PID_Init(&pid.dist, p / 1000.0f, i / 100.0f, d / 1000.0f);
    save_pid();
}
FINSH_FUNCTION_EXPORT(set_ld, set the value of pid in line track dist);

//注册巡线任务
void line_register()
{
    GPIO_InitTypeDef gpio_init;
    
    GPIO_StructInit(&gpio_init);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
    gpio_init.GPIO_Mode=GPIO_Mode_IN;
    gpio_init.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_3|GPIO_Pin_4;
    gpio_init.GPIO_PuPd=GPIO_PuPd_DOWN;
    GPIO_Init(GPIOE,&gpio_init);
    gpio_init.GPIO_Mode=GPIO_Mode_OUT;
    gpio_init.GPIO_PuPd=GPIO_PuPd_NOPULL;
    gpio_init.GPIO_Pin=GPIO_Pin_1|GPIO_Pin_2;
    GPIO_Init(GPIOE,&gpio_init);
    GPIO_WriteBit(GPIOE,GPIO_Pin_2,Bit_RESET);
    GPIO_WriteBit(GPIOE,GPIO_Pin_1,Bit_RESET);
    
    line_task=find_task("default");
    cruise_task=find_task("cruise");
    throw_task =find_task("throw");
    assert_param(line_task!=RT_NULL);
    assert_param(cruise_task!=RT_NULL);
    assert_param(throw_task!=RT_NULL);
    
    line_task->func=line_track;
    cruise_task->func=line_track;
    throw_task->func=line_track;
    
    line_task->var=STRIGHT_MODE;
    cruise_task->var=CRUISE_MODE;
    throw_task->var=THROW_MODE;
    
    init_pid();
}
