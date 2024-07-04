//锟斤拷锟斤拷头锟侥硷拷
#include "Buzzer/buzzer.h"
//写一个测试程序
//锟斤拷锟斤拷锟结构锟斤拷
BuzzerStruct buzzerStr;


//-------------------------------------------------------------------------------------------------------------------
// 锟斤拷锟斤拷锟斤拷锟�     锟斤拷锟斤拷锟斤拷锟斤拷始锟斤拷
// 锟斤拷锟斤拷说锟斤拷     void
// 锟斤拷锟截诧拷锟斤拷     void
//-------------------------------------------------------------------------------------------------------------------
void Buzzer_Init(void)
{
    //锟斤拷始锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷gpio锟节ｏ拷锟斤拷始时为锟酵碉拷平
    gpio_init(P14_6, GPO, 0, GPO_PUSH_PULL);

    //锟斤拷亟峁癸拷锟斤拷锟斤拷锟斤拷始锟斤拷
    buzzerStr.Counter = 0;
    buzzerStr.Cut = 0;
    buzzerStr.Enable = false;
    buzzerStr.Times = 0;
}


//-------------------------------------------------------------------------------------------------------------------
// 锟斤拷锟斤拷锟斤拷锟�     锟斤拷锟狡凤拷锟斤拷锟斤拷锟竭筹拷
// 锟斤拷锟斤拷说锟斤拷     void
// 锟斤拷锟截诧拷锟斤拷     void
//-------------------------------------------------------------------------------------------------------------------
void Buzzer_Timer(void)
{
    //锟斤拷锟秸碉拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟脚猴拷
    if(buzzerStr.Enable)
    {
        //锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
        buzzerStr.Counter++;
        if(buzzerStr.Cut < buzzerStr.Counter)
            buzzerStr.Counter = buzzerStr.Cut;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 锟斤拷锟斤拷锟斤拷锟�     锟斤拷锟斤拷锟斤拷锟斤拷锟狡猴拷锟斤拷
// 锟斤拷锟斤拷说锟斤拷     void
// 锟斤拷锟截诧拷锟斤拷     void
//-------------------------------------------------------------------------------------------------------------------
void Buzzer_Handle(void)
{
    //锟斤拷锟斤拷锟斤拷锟斤拷锟狡ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷没锟斤拷停止锟斤拷锟斤拷锟斤拷
    if(buzzerStr.Enable && !buzzerStr.Silent)
    {
        if(buzzerStr.Times<=0)
        {
            //锟斤拷没锟斤拷锟斤拷锟叫达拷锟斤拷时锟斤拷锟斤拷锟酵碉拷平
            gpio_low(P14_6);
            //失锟杰凤拷锟斤拷锟斤拷
            buzzerStr.Enable = false;
            return;
        }
        else if(buzzerStr.Cut<=buzzerStr.Counter)
        {
            //锟斤拷转锟斤拷锟斤拷锟斤拷锟斤拷锟脚碉拷锟斤拷锟斤拷锟狡�
            gpio_toggle_level(P14_6);
            //锟斤拷转锟斤拷锟斤拷
            buzzerStr.Times--;
            //锟斤拷转时锟斤拷
            buzzerStr.Counter = 0;
        }
    }
    else
        //停止锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
        gpio_low(P14_6);
}


//-------------------------------------------------------------------------------------------------------------------
// 锟斤拷锟斤拷锟斤拷锟�     锟斤拷锟斤拷锟斤拷使锟斤拷
// 锟斤拷锟斤拷说锟斤拷     buzzer锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷模式选锟斤拷
// 锟斤拷锟截诧拷锟斤拷     void
//-------------------------------------------------------------------------------------------------------------------
void Buzzer_Enable(BuzzerEnum buzzer)
{
    //选锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟侥Ｊ�
    switch(buzzer)
    {
        case BuzzerOk:
            buzzerStr.Cut = 70;         //70ms
            buzzerStr.Enable = true;
            buzzerStr.Times = 4;
            break;

        case BuzzerWarnning:
            buzzerStr.Cut = 100;        //100ms
            buzzerStr.Enable = true;
            buzzerStr.Times = 10;
            break;

        case BuzzerSysStart:
            buzzerStr.Cut = 60;         //60ms
            buzzerStr.Enable = true;
            buzzerStr.Times = 6;
            break;

        case BuzzerDing:
            buzzerStr.Cut = 30;         //30ms
            buzzerStr.Enable = true;
            buzzerStr.Times = 2;
            break;

        case BuzzerFinish:
            buzzerStr.Cut = 200;        //200ms
            buzzerStr.Enable = true;
            buzzerStr.Times = 6;
            break;

        case BuzzerVoltage:
            buzzerStr.Cut = 15;         //15ms
            buzzerStr.Enable = true;
            buzzerStr.Times = 10;
            break;
    }

    //锟斤拷占锟斤拷锟斤拷锟�
    buzzerStr.Counter = 0;
}

