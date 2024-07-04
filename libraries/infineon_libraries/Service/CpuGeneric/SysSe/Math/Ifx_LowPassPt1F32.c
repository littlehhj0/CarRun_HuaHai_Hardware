/**
 * \file Ifx_LowPassPt1F32.c
 * \brief Low pass filter PT1
 *
 * \version disabled
 * \copyright Copyright (c) 2013 Infineon Technologies AG. All rights reserved.
 *
 *
 *                                 IMPORTANT NOTICE
 *
 *
 * Use of this file is subject to the terms of use agreed between (i) you or 
 * the company in which ordinary course of business you are acting and (ii) 
 * Infineon Technologies AG or its licensees. If and as long as no such 
 * terms of use are agreed, use of this file is subject to following:


 * Boost Software License - Version 1.0 - August 17th, 2003

 * Permission is hereby granted, free of charge, to any person or 
 * organization obtaining a copy of the software and accompanying 
 * documentation covered by this license (the "Software") to use, reproduce,
 * display, distribute, execute, and transmit the Software, and to prepare
 * derivative works of the Software, and to permit third-parties to whom the 
 * Software is furnished to do so, all subject to the following:

 * The copyright notices in the Software and this entire statement, including
 * the above license grant, this restriction and the following disclaimer, must
 * be included in all copies of the Software, in whole or in part, and all
 * derivative works of the Software, unless such copies or derivative works are
 * solely in the form of machine-executable object code generated by a source
 * language processor.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE 
 * FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.

 *
 */
//��ͨ�˲�
//------------------------------------------------------------------------------
#include "SysSe/Math/Ifx_LowPassPt1F32.h"
//------------------------------------------------------------------------------

/** \brief Set the low pass filter configuration
 *
 * This function sets the low pass filter  configuration and reset the filter output.
 *
 * \param filter Specifies PT1 filter.
 * \param config Specifies the PT1 filter configuration.
 *
 * \return None
 */
void Ifx_LowPassPt1F32_init(Ifx_LowPassPt1F32 *filter, const Ifx_LowPassPt1F32_Config *config)
{
    float32 tStar;
    float32 T = 1 / config->cutOffFrequency;
    tStar       = 1 / (T / config->samplingTime + 1);

    filter->a   = config->gain * tStar;
    filter->b   = tStar;
    filter->out = 0;
}


/** \brief Execute the low pass filter
 * \param filter Specifies PT1 filter.
 * \param input Specifies the filter input.
 *
 * \return Returns the filter output
 */
float32 Ifx_LowPassPt1F32_do(Ifx_LowPassPt1F32 *filter, float32 input)
{
    filter->out = filter->out + filter->a * input - filter->b * filter->out;
    return filter->out;
}
