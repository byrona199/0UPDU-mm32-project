////////////////////////////////////////////////////////////////////////////////
/// @file    i2c.c
/// @author  AE TEAM
/// @brief    In window comparator mode,The transformation results are detected
///           Set the threshold value from 0 to 3V, and connect PB6 and PA0 with
///           jumper cap to see the effect.
////////////////////////////////////////////////////////////////////////////////
/// @attention
///
/// THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
/// CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
/// TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
/// CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
/// HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
/// CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
///
/// <H2><CENTER>&COPY; COPYRIGHT MINDMOTION </CENTER></H2>
////////////////////////////////////////////////////////////////////////////////
// Define to prevent recursive inclusion
#define _I2C_C_

// Files includes
#include "i2c.h"

////////////////////////////////////////////////////////////////////////////////
/// @addtogroup MM32_Hardware_Abstract_Layer
/// @{

////////////////////////////////////////////////////////////////////////////////
/// @addtogroup I2C
/// @{

////////////////////////////////////////////////////////////////////////////////
/// @addtogroup I2C_Exported_Functions
/// @{

volatile u8 I2CRxFlag = 0;
volatile u8 I2CTxFlag = 0;
u8 gData;
u8 recv_err = 0;
extern u8 i2c_err_status;

////////////////////////////////////////////////////////////////////////////////
/// @brief  Initial I2C
/// @note   None.
/// @param  : None.
/// @retval : None.
////////////////////////////////////////////////////////////////////////////////
void I2C_Meter_Init()
{
    I2C1_NVIC_Init();
    I2C1_GPIO_Config();
    //Initializes the I2C master mode
    I2C_MasterModeInit(I2C1, 400000);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  NVIC I2C1 Config
/// @note   None.
/// @param  None.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
static void I2C1_NVIC_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = I2C1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Clock and data bus configuration
/// @note   Keep the bus free which means SCK & SDA is high.
/// @param  : None.
/// @retval : None.
////////////////////////////////////////////////////////////////////////////////
static void I2C1_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(I2C1_SDA_BUSCLK, ENABLE);
    RCC_AHBPeriphClockCmd(I2C1_SCL_BUSCLK, ENABLE);
    GPIO_PinAFConfig(I2C1_SCL_PORT, I2C1_SCL_AFSOURCE, I2C1_SCL_AFMODE);
    GPIO_PinAFConfig(I2C1_SDA_PORT, I2C1_SDA_AFSOURCE, I2C1_SDA_AFMODE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin  = I2C1_SCL_PIN;
    //Set GPIO spped
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    //Keep the bus free which means SCK & SDA is high
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(I2C1_SCL_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin  = I2C1_SDA_PIN;
    GPIO_Init(I2C1_SDA_PORT, &GPIO_InitStructure);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Initializes the I2Cx master mode
/// @note   None.
/// @param  : I2Cx (where x can be 1 or 2 to select the I2C peripheral)
/// @param  : iI2C_speed: I2C speed.
/// @retval : None.
////////////////////////////////////////////////////////////////////////////////
void I2C_MasterModeInit(I2C_TypeDef* I2Cx, u32 uiI2C_speed)
{
    I2C_InitTypeDef I2C_InitStructure;
    I2C_StructInit(&I2C_InitStructure);
    //Enable I2C clock state
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    //Configure I2C as master mode
    I2C_InitStructure.Mode = I2C_CR_MASTER;
    I2C_InitStructure.OwnAddress = 0;
    I2C_InitStructure.Speed = I2C_CR_FAST;
    I2C_InitStructure.ClockSpeed = uiI2C_speed;
    //Initializes the I2Cx peripheral according to the specified
    I2C_Init(I2Cx, &I2C_InitStructure);
    I2C_Cmd(I2Cx, ENABLE);
    I2C_ITConfig( I2C1, I2C_IT_RX_FULL, ENABLE );//|I2C_IT_TX_EMPTY
}

void I2C1_IRQHandler(void)
{
    if(I2C_GetITStatus(I2C1, I2C_IT_TX_EMPTY)) {
        I2C_ClearITPendingBit(I2C1, I2C_IT_TX_EMPTY);
        I2C_ITConfig(I2C1, I2C_IT_TX_EMPTY, DISABLE);
        I2CTxFlag = 1;
    }
    if(I2C_GetITStatus(I2C1, I2C_IT_RX_FULL)) {
        I2C_ClearITPendingBit(I2C1, I2C_IT_RX_FULL);
        gData = I2C_ReceiveData(I2C1);
        I2CRxFlag = 1;
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Set the device address
/// @note   None.
/// @param  : I2Cx(where x can be 1 or 2 to select the I2C peripheral)
/// @param  : deviceaddr(device address).
/// @retval : None.
////////////////////////////////////////////////////////////////////////////////
void I2C_SetDeviceAddr(I2C_TypeDef* I2Cx, u8 deviceaddr)
{
    //Disable I2C
    I2C_Cmd(I2Cx, DISABLE);
    //Set the device address
    I2C_Send7bitAddress(I2Cx, deviceaddr, I2C_Direction_Transmitter);
    //Enable I2C
    I2C_Cmd(I2Cx, ENABLE);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Read a byte.
/// @note   None.
/// @param  : I2Cx(where x can be 1 or 2 to select the I2C peripheral).
/// @retval : None.
////////////////////////////////////////////////////////////////////////////////
static u8 ReadByte(I2C_TypeDef* I2Cx)
{
    u8 temp;
    u8 timeout = 0xFF;

    I2CRxFlag = 0;
    I2C_ReadCmd(I2C1);
    while((I2CRxFlag == 0) && (timeout > 0))
    {
        timeout--;
    }
    if(timeout > 0)
    {
        recv_err = 0;
        temp = gData;
        return (u8)temp;
    }
    else
    {
        recv_err = 1;
        return 0;
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Recv data.
/// @note   None.
/// @param  : receive bytes.
/// @retval : 0 = success, 1 = fail.
////////////////////////////////////////////////////////////////////////////////
u8 RecvData(u8 len)
{
    u8 timeout = 0xFF;
    int i;
    
    //memset(I2CRxBuff, 0, sizeof(I2CRxBuff));
    // recv data
    for(i = 0; i < len; i++) {
        I2CRxBuff[i] = ReadByte(I2C1);
        if(recv_err == 1)
            return 1;
            
    }
    // stop transmit
    I2C_GenerateSTOP(I2C1, ENABLE);
    // check stop
    while(((I2C_GetITStatus(I2C1, I2C_IT_STOP_DET)) == 0) && (timeout > 0))
    {
        timeout--;
    }
    while(i--); //add delay
    if(timeout > 0)
        return 0;
    else
        return 1;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Send a byte.
/// @note   None.
/// @param  : send byte.
/// @retval : 0 = success, 1 = fail.
////////////////////////////////////////////////////////////////////////////////
u8 SendByte(u8 data)
{
    u8 timeout = 0xFF;
    //Send data
    I2C_SendData(I2C1, data);
    //Checks whether transmit FIFO completely empty or not
    //while(I2C_GetFlagStatus(I2C1, I2C_STATUS_FLAG_TFE) == 0);
    while((I2C_GetFlagStatus(I2C1, I2C_STATUS_FLAG_TFE) == 0) && (timeout > 0))
    {
        timeout--;
    }
    if(timeout > 0)
        return 0;
    else
        return 1;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Send command.
/// @note   None.
/// @param  : Command high and low byte.
/// @retval : 0 = success, 1 = fail.
////////////////////////////////////////////////////////////////////////////////
u8 SendCommand(u8 len, u8 CmdH, u8 CmdL)
{
    u8 err = 0;
    u8 i = 0x30;
    u8 timeout = 0xFF;
    //Send data
    err = SendByte(len); // length
    if(err == 1)
        return 1;
    err = SendByte(CmdH);
    if(err == 1)
        return 1;
    err = SendByte(CmdL);
    if(err == 1)
        return 1;

    // stop transmit
    I2C_GenerateSTOP(I2C1, ENABLE);
    // check stop
    while(((I2C_GetITStatus(I2C1, I2C_IT_STOP_DET)) == 0) && (timeout > 0))
    {
        timeout--;
    }
    while(i--); //add delay
    if(timeout > 0)
        return 0;
    else
        return 1;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Write data to register.
/// @note   None.
/// @param  : Register address high and low byte.
/// @retval : None.
////////////////////////////////////////////////////////////////////////////////
void WriteData(u8 len, u8 CmdH, u8 CmdL)
{
    u8 i = 0;
    u8 j = 0x80;
    //Send data
    SendByte(len); // length
    SendByte(CmdH);
    SendByte(CmdL);
    while(j--); // add delay
    for(i = 0; i < (len + 1); i++)
    {
        SendByte(I2CTxBuff[i]);
    }
    // stop transmit
    I2C_GenerateSTOP(I2C1, ENABLE);
    // check stop
    while((I2C_GetITStatus(I2C1, I2C_IT_STOP_DET)) == 0);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Handle receive data
/// @note   None.
/// @param  Command type
/// @retval 0 = success, 1 = fail.
////////////////////////////////////////////////////////////////////////////////
u8 DataHandler(u16 type)
{
    u8 err = 0;
    u8 iaddrH[3] = {IRMSA_ADDRH, IRMSB_ADDRH, IRMSC_ADDRH};
    u8 iaddrL[3] = {IRMSA_ADDRL, IRMSB_ADDRL, IRMSC_ADDRL};
    u8 waddrH[3] = {ACTIVEPOWERA_ADDRH, ACTIVEPOWERB_ADDRH, ACTIVEPOWERC_ADDRH};
    u8 waddrL[3] = {ACTIVEPOWERA_ADDRL, ACTIVEPOWERB_ADDRL, ACTIVEPOWERC_ADDRL};
    u8 kwaddrH[3] = {CF_COUNTA_ADDRH, CF_COUNTB_ADDRH, CF_COUNTC_ADDRH};
    u8 kwaddrL[3] = {CF_COUNTA_ADDRL, CF_COUNTB_ADDRL, CF_COUNTC_ADDRL};
    int start_index = 0, i = 0;
    u32 value = 0, zcnt = 0, zstart = 0, zstop = 0, scnt = 0;
    
    if(type == VOLTA || type == VOLTB || type == VOLTC || type == VOLTD)
    {
        // Voltage /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //
        //                   [D5, D4, D3, D2, D1, D0] x 1000       [D4, D3, D2, D1] x 1000      [D4, D3, D2, D1] x 125
        // voltage * 1000 =  _______________________________  ~=   ________________________  =  ________________________      =   mV
        //
        //                                 2^24                              2^16                          8192
        // Ignore D5, eliminate D0, D0 = 2^8, 2^(24-8) = 2^16
        //
        // Voltage /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        err = SendCommand(6, VRMS_ADDRH, VRMS_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        value = (I2CRxBuff[4] << 24) | (I2CRxBuff[3] << 16) | (I2CRxBuff[2] << 8) | I2CRxBuff[1];
        value = (value*125)/8192;

        voltageVA = voltageVB = voltageVC = voltageVD = (float)value/1000;
    }
    else if(type == HZ)
    {
        // Accumulate Energy ///////////////////////////////////////////////////////////////////////////////////////////////////////
        //
        // Frequency value = (((ZccCnt - 1) / 2) / ((ZccStop - ZccStart) / SampleCnt))
        //
        // Accumulate Energy ///////////////////////////////////////////////////////////////////////////////////////////////////////

        err = SendCommand(6, ZCCCNT_ADDRH, ZCCCNT_ADDRL);
        if(err == 1)
            return 1;        
        err = RecvData(6);
        if(err == 1)
            return 1;
        zcnt = (I2CRxBuff[3] << 24) | (I2CRxBuff[2] << 16) | (I2CRxBuff[1] << 8) | I2CRxBuff[0];

        osDelay(5);

        err = SendCommand(6, ZCCSTART_ADDRH, ZCCSTART_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        zstart = (I2CRxBuff[3] << 24) | (I2CRxBuff[2] << 16) | (I2CRxBuff[1] << 8) | I2CRxBuff[0];

        osDelay(5);

        err = SendCommand(6, ZCCSTOP_ADDRH, ZCCSTOP_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        zstop = (I2CRxBuff[3] << 24) | (I2CRxBuff[2] << 16) | (I2CRxBuff[1] << 8) | I2CRxBuff[0];

        osDelay(5);

        err = SendCommand(2, SAMPLECNT_ADDRH, SAMPLECNT_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(2);
        if(err == 1)
            return 1;
        scnt = I2CRxBuff[1] << 8 | I2CRxBuff[0];

        frequencyHz = ((float)zcnt - 1) / 2;
        frequencyHz = frequencyHz / (((float)zstop - (float)zstart) / (float)scnt);
    }
    else if(type == CURRA || type == CURRB || type == CURRC || type == CURRD)
    {
        // Current /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //
        //                   [D5, D4, D3, D2, D1, D0] x 1000       [D5, D4, D3, D2] x 1000      [D5, D4, D3, D2] x 125
        // Current * 1000 =  _______________________________  ~=   ________________________  =  ________________________      =   mA
        //
        //                                 2^30                              2^14                          2^11
        // Eliminate D1 D0, D1 D0 = 2^16, 2^(30-16) = 2^14
        //
        // Current /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // total current do not get anymore

        if(type == CURRB)
            start_index = 3;
        else if(type == CURRC)
            start_index = 6;
        else if(type == CURRD)
            start_index = 9;

        for(i = 0; i < 3; i++)
        {
            err = SendCommand(6, iaddrH[i], iaddrL[i]);
            if(err == 1)
                return 1;
            err = RecvData(6);
            if(err == 1)
                return 1;
            value = (I2CRxBuff[5] << 24) | (I2CRxBuff[4] << 16) | (I2CRxBuff[3] << 8) | I2CRxBuff[2];
            value = (value*125)/2048;
            o_current[start_index + i] = (float)value/1000;
            osDelay(5);
        }
    }
#if 0
    else if(type == CURRA)
    {
        // Current /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //
        //                   [D5, D4, D3, D2, D1, D0] x 1000       [D5, D4, D3, D2] x 1000      [D5, D4, D3, D2] x 125
        // Current * 1000 =  _______________________________  ~=   ________________________  =  ________________________      =   mA
        //
        //                                 2^30                              2^14                          2^11
        // Eliminate D1 D0, D1 D0 = 2^16, 2^(30-16) = 2^14
        //
        // Current /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // total current do not get anymore

        // outlet 1
        err = SendCommand(6, IRMSA_ADDRH, IRMSA_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        value = (I2CRxBuff[5] << 24) | (I2CRxBuff[4] << 16) | (I2CRxBuff[3] << 8) | I2CRxBuff[2];
        value = (value*125)/2048;
        o_current[0]= (float)value/1000;
        osDelay(5);

        // outlet 2
        err = SendCommand(6, IRMSB_ADDRH, IRMSB_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        value = (I2CRxBuff[5] << 24) | (I2CRxBuff[4] << 16) | (I2CRxBuff[3] << 8) | I2CRxBuff[2];
        value = (value*125)/2048;
        o_current[1] = (float)value/1000;
        osDelay(5);

        // outlet 3
        err = SendCommand(6, IRMSC_ADDRH, IRMSC_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        value = (I2CRxBuff[5] << 24) | (I2CRxBuff[4] << 16) | (I2CRxBuff[3] << 8) | I2CRxBuff[2];
        value = (value*125)/2048;
        o_current[2] = (float)value/1000;
    }
    else if(type == CURRB)
    {
        // outlet 4
        err = SendCommand(6, IRMSA_ADDRH, IRMSA_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        value = (I2CRxBuff[5] << 24) | (I2CRxBuff[4] << 16) | (I2CRxBuff[3] << 8) | I2CRxBuff[2];
        value = (value*125)/2048;
        o_current[3]= (float)value/1000;
        osDelay(5);

        // outlet 5
        err = SendCommand(6, IRMSB_ADDRH, IRMSB_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        value = (I2CRxBuff[5] << 24) | (I2CRxBuff[4] << 16) | (I2CRxBuff[3] << 8) | I2CRxBuff[2];
        value = (value*125)/2048;
        o_current[4] = (float)value/1000;
        osDelay(5);
        
        // outlet 6
        err = SendCommand(6, IRMSC_ADDRH, IRMSC_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        value = (I2CRxBuff[5] << 24) | (I2CRxBuff[4] << 16) | (I2CRxBuff[3] << 8) | I2CRxBuff[2];
        value = (value*125)/2048;
        o_current[5] = (float)value/1000;
    }
    else if(type == CURRC)
    {
        // outlet 7
        err = SendCommand(6, IRMSA_ADDRH, IRMSA_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        value = (I2CRxBuff[5] << 24) | (I2CRxBuff[4] << 16) | (I2CRxBuff[3] << 8) | I2CRxBuff[2];
        value = (value*125)/2048;
        o_current[6]= (float)value/1000;
        osDelay(5);
        
        // outlet 8
        err = SendCommand(6, IRMSB_ADDRH, IRMSB_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        value = (I2CRxBuff[5] << 24) | (I2CRxBuff[4] << 16) | (I2CRxBuff[3] << 8) | I2CRxBuff[2];
        value = (value*125)/2048;
        o_current[7] = (float)value/1000;
        osDelay(5);
        
        // outlet 9
        err = SendCommand(6, IRMSC_ADDRH, IRMSC_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        value = (I2CRxBuff[5] << 24) | (I2CRxBuff[4] << 16) | (I2CRxBuff[3] << 8) | I2CRxBuff[2];
        value = (value*125)/2048;
        o_current[8] = (float)value/1000;
    }
#endif
    else if(type == WATTA || type == WATTB || type == WATTC || type == WATTD)
    {
        // Watt ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //
        //                   [D5, D4, D3, D2, D1, D0] x 1000       [D4, D3, D2, D1] x 1000      [D4, D3, D2, D1] x 125
        // watt * 1000 =  _______________________________  ~=   ________________________  =  ________________________      =   mW
        //
        //                                 2^24                              2^8                          2^5
        // Eliminate D1 D0, D1 D0 = 2^16, 2^(24-16) = 2^8
        //
        // Watt ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if(type == WATTB)
            start_index = 3;
        else if(type == WATTC)
            start_index = 6;
        else if(type == WATTD)
            start_index = 9;

        for(i = 0; i < 3; i++)
        {
            err = SendCommand(6, waddrH[i], waddrL[i]);
            if(err == 1)
                return 1;
            err = RecvData(6);
            if(err == 1)
                return 1;
            value = (I2CRxBuff[5] << 24) | (I2CRxBuff[4] << 16) | (I2CRxBuff[3] << 8) | I2CRxBuff[2];
            value = (value*125)/32;
            o_kw[start_index + i] = (float)value/1000;
            osDelay(5);
        }
    }
#if 0
else if(type == WATTA)
    {
        // Watt ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //
        //                   [D5, D4, D3, D2, D1, D0] x 1000       [D4, D3, D2, D1] x 1000      [D4, D3, D2, D1] x 125
        // watt * 1000 =  _______________________________  ~=   ________________________  =  ________________________      =   mW
        //
        //                                 2^24                              2^8                          2^5
        // Eliminate D1 D0, D1 D0 = 2^16, 2^(24-16) = 2^8
        //
        // Watt ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // total watt

        /*err = SendCommand(6, ACTIVEPOWERA_ADDRH, ACTIVEPOWERA_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        value = (I2CRxBuff[5] << 24) | (I2CRxBuff[4] << 16) | (I2CRxBuff[3] << 8) | I2CRxBuff[2];
        value = (value*125)/32;
        wattW = (float)value/1000;
        
        osDelay(5);*/
        
        // outlet 1
        err = SendCommand(6, ACTIVEPOWERB_ADDRH, ACTIVEPOWERB_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        value = (I2CRxBuff[5] << 24) | (I2CRxBuff[4] << 16) | (I2CRxBuff[3] << 8) | I2CRxBuff[2];
        value = (value*125)/32;
        o_kw[0] = (float)value/1000;
        
        osDelay(5);
        
        // outlet 2
        err = SendCommand(6, ACTIVEPOWERC_ADDRH, ACTIVEPOWERC_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        value = (I2CRxBuff[5] << 24) | (I2CRxBuff[4] << 16) | (I2CRxBuff[3] << 8) | I2CRxBuff[2];
        value = (value*125)/32;
        o_kw[1] = (float)value/1000;
    }
    else if(type == WATTB)
    {
        // outlet 3
        err = SendCommand(6, ACTIVEPOWERA_ADDRH, ACTIVEPOWERA_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        value = (I2CRxBuff[5] << 24) | (I2CRxBuff[4] << 16) | (I2CRxBuff[3] << 8) | I2CRxBuff[2];
        value = (value*125)/32;
        o_kw[2] = (float)value/1000;
        
        osDelay(5);
        
        // outlet 4
        err = SendCommand(6, ACTIVEPOWERB_ADDRH, ACTIVEPOWERB_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        value = (I2CRxBuff[5] << 24) | (I2CRxBuff[4] << 16) | (I2CRxBuff[3] << 8) | I2CRxBuff[2];
        value = (value*125)/32;
        o_kw[3] = (float)value/1000;
        
        osDelay(5);
        
        // outlet 5
        err = SendCommand(6, ACTIVEPOWERC_ADDRH, ACTIVEPOWERC_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        value = (I2CRxBuff[5] << 24) | (I2CRxBuff[4] << 16) | (I2CRxBuff[3] << 8) | I2CRxBuff[2];
        value = (value*125)/32;
        o_kw[4] = (float)value/1000;
    }
    else if(type == WATTC)
    {
        // outlet 6
        err = SendCommand(6, ACTIVEPOWERA_ADDRH, ACTIVEPOWERA_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        value = (I2CRxBuff[5] << 24) | (I2CRxBuff[4] << 16) | (I2CRxBuff[3] << 8) | I2CRxBuff[2];
        value = (value*125)/32;
        o_kw[5] = (float)value/1000;
        
        osDelay(5);
        
        // outlet 7
        err = SendCommand(6, ACTIVEPOWERB_ADDRH, ACTIVEPOWERB_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        value = (I2CRxBuff[5] << 24) | (I2CRxBuff[4] << 16) | (I2CRxBuff[3] << 8) | I2CRxBuff[2];
        value = (value*125)/32;
        o_kw[6] = (float)value/1000;
        
        osDelay(5);
        
        // outlet 8
        err = SendCommand(6, ACTIVEPOWERC_ADDRH, ACTIVEPOWERC_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        value = (I2CRxBuff[5] << 24) | (I2CRxBuff[4] << 16) | (I2CRxBuff[3] << 8) | I2CRxBuff[2];
        value = (value*125)/32;
        o_kw[7] = (float)value/1000;
    }
#endif
    else if(type == PFA || type == PFB || type == PFC || type == PFD)
    {
        // POWER FACTOR ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //
        // PF value = (ActivePower) / ( Vrms * Irms) 
        //
        // POWER FACTOR ////////////////////////////////////////////////////////////////////////////////////////////////////////////

        if(type == PFB)
            start_index = 3;
        else if(type == PFC)
            start_index = 6;
        else if(type == PFD)
            start_index = 9;

        for(i = 0; i < 3; i++)
        {
            if(voltageVA == 0 | o_current[start_index + i] == 0) {
                o_pf[start_index + i] = 0;
            }
            else {
                o_pf[start_index + i] = o_kw[start_index + i] / voltageVA;
                o_pf[start_index + i] = o_pf[start_index + i] / o_current[start_index + i];
            }
        }
    }
#if 0
    else if(type == PFA)
    {
        // POWER FACTOR ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //
        // PF value = (ActivePower) / ( Vrms * Irms) 
        //
        // POWER FACTOR ////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // total pf

        /*if(voltageVA == 0 | currentA == 0) {
            pf = 0;
        }
        else {
            pf = wattW/voltageVA;
            pf = pf/currentA;
        }*/

        // outlet 1
        if(voltageVA == 0 | o_current[0] == 0) {
            o_pf[0] = 0;
        }
        else {
            o_pf[0] = o_kw[0] / voltageVA;
            o_pf[0] = o_pf[0] / o_current[0];
        }
        // outlet 2
        if(voltageVA == 0 | o_current[1] == 0) {
            o_pf[1] = 0;
        }
        else {
            o_pf[1] = o_kw[1] / voltageVA;
            o_pf[1] = o_pf[1] / o_current[1];
        }
    }
    else if(type == PFB)
    {
        // outlet 3
        if(voltageVB == 0 | o_current[2] == 0) {
            o_pf[2] = 0;
        }
        else {
            o_pf[2] = o_kw[2] / voltageVB;
            o_pf[2] = o_pf[2] / o_current[2];
        }
        // outlet 4
        if(voltageVB == 0 | o_current[3] == 0) {
            o_pf[3] = 0;
        }
        else {
            o_pf[3] = o_kw[3] / voltageVB;
            o_pf[3] = o_pf[3] / o_current[3];
        }
        // outlet 5
        if(voltageVB == 0 | o_current[4] == 0) {
            o_pf[4] = 0;
        }
        else {
            o_pf[4] = o_kw[4] / voltageVB;
            o_pf[4] = o_pf[4] / o_current[4];
        }
    }
    else if(type == PFC)
    {
        // outlet 6
        if(voltageVC == 0 | o_current[5] == 0) {
            o_pf[5] = 0;
        }
        else {
            o_pf[5] = o_kw[5] / voltageVC;
            o_pf[5] = o_pf[5] / o_current[5];
        }
        // outlet 7
        if(voltageVC == 0 | o_current[6] == 0) {
            o_pf[6] = 0;
        }
        else {
            o_pf[6] = o_kw[6] / voltageVC;
            o_pf[6] = o_pf[6] / o_current[6];
        }
        // outlet 8
        if(voltageVC == 0 | o_current[7] == 0) {
            o_pf[7] = 0;
        }
        else {
            o_pf[7] = o_kw[7] / voltageVC;
            o_pf[7] = o_pf[7] / o_current[7];
        }
    }
#endif
    else if(type == KWHA || type == KWHB || type == KWHC || type == KWHD)
    {
        if(type == KWHB)
            start_index = 3;
        else if(type == KWHC)
            start_index = 6;
        else if(type == KWHD)
            start_index = 9;

        for(i = 0; i < 3; i++)
        {
            err = SendCommand(6, kwaddrH[i], kwaddrL[i]);
            if(err == 1)
                return 1;
            err = RecvData(6);
            if(err == 1)
                return 1;
            o_kwh[start_index + i] = (I2CRxBuff[3] << 24) | (I2CRxBuff[2] << 16) | (I2CRxBuff[1] << 8) | I2CRxBuff[0];
            o_kwh[start_index + i] = o_kwh[start_index + i] / 1000;
            o_kwh[start_index + i] = o_kwh[start_index + i] * 0.3125;
        }
        
        osDelay(5);
    }
#if 0
    else if(type == KWHA)
    {
        // Accumulate Energy ///////////////////////////////////////////////////////////////////////////////////////////////////////
        //
        // CF_Count * 0.3125 / 1000 = [D5, D4, D3, D2, D1, D0] x 1000  ~=  [D3, D2, D1, D0] x 1000 = KWH
        // Accumulate Energy(WH) Value  = CF_Count *0.3125
        //
        // Accumulate Energy ///////////////////////////////////////////////////////////////////////////////////////////////////////

        // total kwh

        /*err = SendCommand(6, CF_COUNTA_ADDRH, CF_COUNTA_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        kwh = (I2CRxBuff[3] << 24) | (I2CRxBuff[2] << 16) | (I2CRxBuff[1] << 8) | I2CRxBuff[0];
        kwh = kwh / 1000;
        kwh = kwh * 0.3125;
        
        osDelay(5);*/

        // outlet 1
        err = SendCommand(6, CF_COUNTB_ADDRH, CF_COUNTB_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        o_kwh[0] = (I2CRxBuff[3] << 24) | (I2CRxBuff[2] << 16) | (I2CRxBuff[1] << 8) | I2CRxBuff[0];
        o_kwh[0] = o_kwh[0] / 1000;
        o_kwh[0] = o_kwh[0] * 0.3125;
        
        osDelay(5);
        
        // outlet 2
        err = SendCommand(6, CF_COUNTC_ADDRH, CF_COUNTC_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        o_kwh[1] = (I2CRxBuff[3] << 24) | (I2CRxBuff[2] << 16) | (I2CRxBuff[1] << 8) | I2CRxBuff[0];
        o_kwh[1] = o_kwh[1] / 1000;
        o_kwh[1] = o_kwh[1] * 0.3125;
    }
    else if(type == KWHB)
    {
        // outlet 3
        err = SendCommand(6, CF_COUNTA_ADDRH, CF_COUNTA_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        o_kwh[2] = (I2CRxBuff[3] << 24) | (I2CRxBuff[2] << 16) | (I2CRxBuff[1] << 8) | I2CRxBuff[0];
        o_kwh[2] = o_kwh[2] / 1000;
        o_kwh[2] = o_kwh[2] * 0.3125;
        
        osDelay(5);
        
        // outlet 4
        err = SendCommand(6, CF_COUNTB_ADDRH, CF_COUNTB_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        o_kwh[3] = (I2CRxBuff[3] << 24) | (I2CRxBuff[2] << 16) | (I2CRxBuff[1] << 8) | I2CRxBuff[0];
        o_kwh[3] = o_kwh[3] / 1000;
        o_kwh[3] = o_kwh[3] * 0.3125;
        
        osDelay(5);
        
        // outlet 5
        err = SendCommand(6, CF_COUNTC_ADDRH, CF_COUNTC_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        o_kwh[4] = (I2CRxBuff[3] << 24) | (I2CRxBuff[2] << 16) | (I2CRxBuff[1] << 8) | I2CRxBuff[0];
        o_kwh[4] = o_kwh[4] / 1000;
        o_kwh[4] = o_kwh[4] * 0.3125;
    }
    else if(type == KWHC)
    {
        // outlet 6
        err = SendCommand(6, CF_COUNTA_ADDRH, CF_COUNTA_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        o_kwh[5] = (I2CRxBuff[3] << 24) | (I2CRxBuff[2] << 16) | (I2CRxBuff[1] << 8) | I2CRxBuff[0];
        o_kwh[5] = o_kwh[5] / 1000;
        o_kwh[5] = o_kwh[5] * 0.3125;
        
        osDelay(5);
        
        // outlet 7
        err = SendCommand(6, CF_COUNTB_ADDRH, CF_COUNTB_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        o_kwh[6] = (I2CRxBuff[3] << 24) | (I2CRxBuff[2] << 16) | (I2CRxBuff[1] << 8) | I2CRxBuff[0];
        o_kwh[6] = o_kwh[6] / 1000;
        o_kwh[6] = o_kwh[6] * 0.3125;
        
        osDelay(5);
        
        // outlet 8
        err = SendCommand(6, CF_COUNTC_ADDRH, CF_COUNTC_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(6);
        if(err == 1)
            return 1;
        o_kwh[7] = (I2CRxBuff[3] << 24) | (I2CRxBuff[2] << 16) | (I2CRxBuff[1] << 8) | I2CRxBuff[0];
        o_kwh[7] = o_kwh[7] / 1000;
        o_kwh[7] = o_kwh[7] * 0.3125;
    }
#endif

#if 0
    /* TRIP
    0x3916
    [7] LTIB_TRIP
    [6] LTIA_TRIP
    [5] AVM_TRIP
    [4] OTP_TRIP
    [3] INSTB_TRIP
    [2] INSTA_TRIP
    [1] INSTC_TRIP
    [0]
    0x3917
    [7]
    [6]
    [5]
    [4] STIC_TRIP
    [3] STIB_TRIP
    [2] STIA_TRIP
    [1]
    [0] LTIC_TRIP
    */
    else if(type == TRIPA)
    {
        err = SendCommand(2, TRIP_ADDRH, TRIP_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(2);
        if(err == 1)
            return 1;

        // total trip trip[0] = 0000 01(LI TRIP)1(ST TRIP)1(INST TRIP)
#if 0
        if((I2CRxBuff[0] & 0x04) == 0x04)       // if get trip INSTA_TRIP
        {
            if((trip[0] | 0xFE) == 0xFE)        // if no trip to trip, send trip event
            {
                trip[0] = trip[0] | 0x01;
                trip_flag = 1;
            }
        }
        if((I2CRxBuff[1] & 0x04) == 0x04)       // if get trip STIA_TRIP
        {
            if((trip[0] | 0xFD) == 0xFD)
            {
                trip[0] = trip[0] | 0x02;
                trip_flag = 1;
            }
        }
        if((I2CRxBuff[0] & 0x40) == 0x40)       // if get trip LTIA_TRIP
        {
            if((trip[0] | 0xFB) == 0xFB)
            {
                trip[0] = trip[0] | 0x04;
                trip_flag = 1;
            }
        }
#endif
        // outlet 1 trip[1] = 0000 01(LI TRIP)1(ST TRIP)1(INST TRIP)
        if((I2CRxBuff[0] & 0x08) == 0x08)       // if get trip INSTB_TRIP
        {
            if((trip[1] | 0xFE) == 0xFE)        // if no trip to trip, send trip event
            {
                trip[1] = trip[1] | 0x01;
                trip_flag = 1;
            }
            curr_state[0] = '0';                // if trip, relay will off, change mm32 relay state
        }
        if((I2CRxBuff[1] & 0x08) == 0x08)        // if get trip STIB_TRIP
        {
            if((trip[1] | 0xFD) == 0xFD)
            {
                trip[1] = trip[1] | 0x02;
                trip_flag = 1;
            }
            curr_state[0] = '0';
        }
        if((I2CRxBuff[0] & 0x80) == 0x80)        // if get trip LTIB_TRIP
        {
            if((trip[1] | 0xFB) == 0xFB)
            {
                trip[1] = trip[1] | 0x04;
                trip_flag = 1;
            }
            curr_state[0] = '0';
        }

        // outlet 2 trip[2] = 0000 01(LI TRIP)1(ST TRIP)1(INST TRIP)
        if((I2CRxBuff[0] & 0x02) == 0x02)        // if get trip INSTC_TRIP
        {
            if((trip[2] | 0xFE) == 0xFE)        // if no trip to trip, send trip event
            {
                trip[2] = trip[2] | 0x01;
                trip_flag = 1;
            }
            curr_state[1] = '0';
        }
        if((I2CRxBuff[1] & 0x10) == 0x10)        // if get trip STIC_TRIP
        {
            if((trip[2] | 0xFD) == 0xFD)
            {
                trip[2] = trip[2] | 0x02;
                trip_flag = 1;
            }
            curr_state[1] = '0';
        }
        if((I2CRxBuff[1] & 0x01) == 0x01)        // if get trip LTIC_TRIP
        {
            if((trip[2] | 0xFB) == 0xFB)
            {
                trip[2] = trip[2] | 0x04;
                trip_flag = 1;
            }
            curr_state[1] = '0';
        }
    }
    else if(type == TRIPB)
    {
        err = SendCommand(2, TRIP_ADDRH, TRIP_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(2);
        if(err == 1)
            return 1;

        // outlet 3 trip[3] = 0000 01(LI TRIP)1(ST TRIP)1(INST TRIP)
        if((I2CRxBuff[0] & 0x04) == 0x04)       // if get trip INSTA_TRIP
        {
            if((trip[3] | 0xFE) == 0xFE)        // if no trip to trip, send trip event
            {
                trip[3] = trip[3] | 0x01;
                trip_flag = 1;
            }
            curr_state[2] = '0';
        }
        if((I2CRxBuff[1] & 0x04) == 0x04)       // if get trip STIA_TRIP
        {
            if((trip[3] | 0xFD) == 0xFD)
            {
                trip[3] = trip[3] | 0x02;
                trip_flag = 1;
            }
            curr_state[2] = '0';
        }
        if((I2CRxBuff[0] & 0x40) == 0x40)       // if get trip LTIA_TRIP
        {
            if((trip[3] | 0xFB) == 0xFB)
            {
                trip[3] = trip[3] | 0x04;
                trip_flag = 1;
            }
            curr_state[2] = '0';
        }

        // outlet 4 trip[4] = 0000 01(LI TRIP)1(ST TRIP)1(INST TRIP)
        if((I2CRxBuff[0] & 0x08) == 0x08)       // if get trip INSTB_TRIP
        {
            if((trip[4] | 0xFE) == 0xFE)        // if no trip to trip, send trip event
            {
                trip[4] = trip[4] | 0x01;
                trip_flag = 1;
            }
            curr_state[3] = '0';                // if trip, relay will off, change mm32 relay state
        }
        if((I2CRxBuff[1] & 0x08) == 0x08)        // if get trip STIB_TRIP
        {
            if((trip[4] | 0xFD) == 0xFD)
            {
                trip[4] = trip[4] | 0x02;
                trip_flag = 1;
            }
            curr_state[3] = '0';
        }
        if((I2CRxBuff[0] & 0x80) == 0x80)        // if get trip LTIB_TRIP
        {
            if((trip[4] | 0xFB) == 0xFB)
            {
                trip[4] = trip[4] | 0x04;
                trip_flag = 1;
            }
            curr_state[3] = '0';
        }

        // outlet 5 trip[5] = 0000 01(LI TRIP)1(ST TRIP)1(INST TRIP)
        if((I2CRxBuff[0] & 0x02) == 0x02)        // if get trip INSTC_TRIP
        {
            if((trip[5] | 0xFE) == 0xFE)        // if no trip to trip, send trip event
            {
                trip[5] = trip[5] | 0x01;
                trip_flag = 1;
            }
            curr_state[4] = '0';
        }
        if((I2CRxBuff[1] & 0x10) == 0x10)        // if get trip STIC_TRIP
        {
            if((trip[5] | 0xFD) == 0xFD)
            {
                trip[5] = trip[5] | 0x02;
                trip_flag = 1;
            }
            curr_state[4] = '0';
        }
        if((I2CRxBuff[1] & 0x01) == 0x01)        // if get trip LTIC_TRIP
        {
            if((trip[5] | 0xFB) == 0xFB)
            {
                trip[5] = trip[5] | 0x04;
                trip_flag = 1;
            }
            curr_state[4] = '0';
        }
    }
    else if(type == TRIPC)
    {
        err = SendCommand(2, TRIP_ADDRH, TRIP_ADDRL);
        if(err == 1)
            return 1;
        err = RecvData(2);
        if(err == 1)
            return 1;

        // outlet 6 trip[6] = 0000 01(LI TRIP)1(ST TRIP)1(INST TRIP)
        if((I2CRxBuff[0] & 0x04) == 0x04)       // if get trip INSTA_TRIP
        {
            if((trip[6] | 0xFE) == 0xFE)        // if no trip to trip, send trip event
            {
                trip[6] = trip[6] | 0x01;
                trip_flag = 1;
            }
            curr_state[5] = '0';
        }
        if((I2CRxBuff[1] & 0x04) == 0x04)       // if get trip STIA_TRIP
        {
            if((trip[6] | 0xFD) == 0xFD)
            {
                trip[6] = trip[6] | 0x02;
                trip_flag = 1;
            }
            curr_state[5] = '0';
        }
        if((I2CRxBuff[0] & 0x40) == 0x40)       // if get trip LTIA_TRIP
        {
            if((trip[6] | 0xFB) == 0xFB)
            {
                trip[6] = trip[6] | 0x04;
                trip_flag = 1;
            }
            curr_state[5] = '0';
        }

        // outlet 7 trip[7] = 0000 01(LI TRIP)1(ST TRIP)1(INST TRIP)
        if((I2CRxBuff[0] & 0x08) == 0x08)       // if get trip INSTB_TRIP
        {
            if((trip[7] | 0xFE) == 0xFE)        // if no trip to trip, send trip event
            {
                trip[7] = trip[7] | 0x01;
                trip_flag = 1;
            }
            curr_state[6] = '0';                // if trip, relay will off, change mm32 relay state
        }
        if((I2CRxBuff[1] & 0x08) == 0x08)        // if get trip STIB_TRIP
        {
            if((trip[7] | 0xFD) == 0xFD)
            {
                trip[7] = trip[7] | 0x02;
                trip_flag = 1;
            }
            curr_state[6] = '0';
        }
        if((I2CRxBuff[0] & 0x80) == 0x80)        // if get trip LTIB_TRIP
        {
            if((trip[7] | 0xFB) == 0xFB)
            {
                trip[7] = trip[7] | 0x04;
                trip_flag = 1;
            }
            curr_state[6] = '0';
        }

        // outlet 8 trip[8] = 0000 01(LI TRIP)1(ST TRIP)1(INST TRIP)
        if((I2CRxBuff[0] & 0x02) == 0x02)        // if get trip INSTC_TRIP
        {
            if((trip[8] | 0xFE) == 0xFE)        // if no trip to trip, send trip event
            {
                trip[8] = trip[8] | 0x01;
                trip_flag = 1;
            }
            curr_state[7] = '0';
        }
        if((I2CRxBuff[1] & 0x10) == 0x10)        // if get trip STIC_TRIP
        {
            if((trip[8] | 0xFD) == 0xFD)
            {
                trip[8] = trip[8] | 0x02;
                trip_flag = 1;
            }
            curr_state[7] = '0';
        }
        if((I2CRxBuff[1] & 0x01) == 0x01)        // if get trip LTIC_TRIP
        {
            if((trip[8] | 0xFB) == 0xFB)
            {
                trip[8] = trip[8] | 0x04;
                trip_flag = 1;
            }
            curr_state[7] = '0';
        }
    }
#endif
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Switch relay on / off
/// @note   None.
/// @param  Relay
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void SwitchRelay(u8 *data)
{
    // Relay 1  PB1
    // Relay 2  PB0
    // Relay 3  PA7
    // Relay 4  PA6
    // Relay 5  PA5
    // Relay 6  PA4
    // Relay 7  PB2
    // Relay 8  PA8
    // Relay 9  PA11
    // Relay 10 PA12
    // Relay 11 PA15
    // Relay 12 PB3

    // Relay 1 (PB1)
    if(curr_state[0] != outlets_state[0]) {               // outlet state change
        if (outlets_state[0] == '1') {
            GPIO_SetBits(GPIOB, GPIO_Pin_1);
            curr_state[0] = outlets_state[0];             // mm32 relay state record
        } else {
            GPIO_ResetBits(GPIOB, GPIO_Pin_1);
            curr_state[0] = outlets_state[0];             // mm32 relay state record
        }
        osDelay(800);                                    // delay 890 ms to set next relay
    }
    // Relay 2 (PB0)
    if(curr_state[1] != outlets_state[1]) {               // outlet state change
        if (outlets_state[1] == '1') {
            GPIO_SetBits(GPIOB, GPIO_Pin_0);
            curr_state[1] = outlets_state[1];             // mm32 relay state record
        } else {
            GPIO_ResetBits(GPIOB, GPIO_Pin_0);
            curr_state[1] = outlets_state[1];             // mm32 relay state record
        }
        osDelay(800);                                    // delay 890 ms to set next relay
    }
    // Relay 3 (PA7)
    if(curr_state[2] != outlets_state[2]) {               // outlet state change
        if (outlets_state[2] == '1') {
            GPIO_SetBits(GPIOA, GPIO_Pin_7);
            curr_state[2] = outlets_state[2];             // mm32 relay state record
        } else {
            GPIO_ResetBits(GPIOA, GPIO_Pin_7);
            curr_state[2] = outlets_state[2];             // mm32 relay state record
        }
        osDelay(800);                                    // delay 890 ms to set next relay
    }
    // Relay 4 (PA6)
    if(curr_state[3] != outlets_state[3]) {               // outlet state change
        if (outlets_state[3] == '1') {
            GPIO_SetBits(GPIOA, GPIO_Pin_6);
            curr_state[3] = outlets_state[3];             // mm32 relay state record
        } else {
            GPIO_ResetBits(GPIOA, GPIO_Pin_6);
            curr_state[3] = outlets_state[3];             // mm32 relay state record
        }
        osDelay(800);                                    // delay 890 ms to set next relay
    }
    // Relay 5 (PA5)
    if(curr_state[4] != outlets_state[4]) {               // outlet state change
        if (outlets_state[4] == '1') {
            GPIO_SetBits(GPIOA, GPIO_Pin_5);
            curr_state[4] = outlets_state[4];             // mm32 relay state record
        } else {
            GPIO_ResetBits(GPIOA, GPIO_Pin_5);
            curr_state[4] = outlets_state[4];             // mm32 relay state record
        }
        osDelay(800);                                    // delay 890 ms to set next relay
    }
    // Relay 6 (PA4)
    if(curr_state[5] != outlets_state[5]) {               // outlet state change
        if (outlets_state[5] == '1') {
            GPIO_SetBits(GPIOA, GPIO_Pin_4);
            curr_state[5] = outlets_state[5];             // mm32 relay state record
        } else {
            GPIO_ResetBits(GPIOA, GPIO_Pin_4);
            curr_state[5] = outlets_state[5];             // mm32 relay state record
        }
        osDelay(800);                                    // delay 890 ms to set next relay
    }
    // Relay 7 (PB2)
    if(curr_state[6] != outlets_state[6]) {               // outlet state change
        if (outlets_state[6] == '1') {
            GPIO_SetBits(GPIOB, GPIO_Pin_2);
            curr_state[6] = outlets_state[6];             // mm32 relay state record
        } else {
            GPIO_ResetBits(GPIOB, GPIO_Pin_2);
            curr_state[6] = outlets_state[6];             // mm32 relay state record
        }
        osDelay(800);                                    // delay 890 ms to set next relay
    }
    // Relay 8 (PA8)
    if(curr_state[7] != outlets_state[7]) {               // outlet state change
        if (outlets_state[7] == '1') {
            GPIO_SetBits(GPIOA, GPIO_Pin_8);
            curr_state[7] = outlets_state[7];             // mm32 relay state record
        } else {
            GPIO_ResetBits(GPIOA, GPIO_Pin_8);
            curr_state[7] = outlets_state[7];             // mm32 relay state record
        }
        osDelay(800);                                    // delay 890 ms to set next relay
    }
    // Relay 9 (PA11)
    if(curr_state[8] != outlets_state[8]) {               // outlet state change
        if (outlets_state[8] == '1') {
            GPIO_SetBits(GPIOA, GPIO_Pin_11);
            curr_state[8] = outlets_state[8];             // mm32 relay state record
        } else {
            GPIO_ResetBits(GPIOA, GPIO_Pin_11);
            curr_state[8] = outlets_state[8];             // mm32 relay state record
        }
        osDelay(800);                                    // delay 890 ms to set next relay
    }
    // Relay 10 (PA12)
    if(curr_state[9] != outlets_state[9]) {               // outlet state change
        if (outlets_state[9] == '1') {
            GPIO_SetBits(GPIOA, GPIO_Pin_12);
            curr_state[9] = outlets_state[9];             // mm32 relay state record
        } else {
            GPIO_ResetBits(GPIOA, GPIO_Pin_12);
            curr_state[9] = outlets_state[9];             // mm32 relay state record
        }
        osDelay(800);                                    // delay 890 ms to set next relay
    }
    // Relay 11 (PA15)
    if(curr_state[10] != outlets_state[10]) {               // outlet state change
        if (outlets_state[10] == '1') {
            GPIO_SetBits(GPIOA, GPIO_Pin_15);
            curr_state[10] = outlets_state[10];             // mm32 relay state record
        } else {
            GPIO_ResetBits(GPIOA, GPIO_Pin_15);
            curr_state[10] = outlets_state[10];             // mm32 relay state record
        }
        osDelay(800);                                    // delay 890 ms to set next relay
    }
    // Relay 12 (PB3)
    if(curr_state[11] != outlets_state[11]) {               // outlet state change
        if (outlets_state[11] == '1') {
            GPIO_SetBits(GPIOB, GPIO_Pin_3);
            curr_state[11] = outlets_state[11];             // mm32 relay state record
        } else {
            GPIO_ResetBits(GPIOB, GPIO_Pin_3);
            curr_state[11] = outlets_state[11];             // mm32 relay state record
        }
        osDelay(800);                                    // delay 890 ms to set next relay
    }

    // Check relay 1
    if (GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_1) == Bit_SET) {
        data[1] |= 1 << 7;
    } else {
        data[1] |= 0 << 7;
    }
    // Check relay 2
    if (GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_0) == Bit_SET) {
        data[1] |= 1 << 6;
    } else {
        data[1] |= 0 << 6;
    }
    // Check relay 3
    if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_7) == Bit_SET) {
        data[1] |= 1 << 5;
    } else {
        data[1] |= 0 << 5;
    }
    // Check relay 4
    if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_6) == Bit_SET) {
        data[1] |= 1 << 4;
    } else {
        data[1] |= 0 << 4;
    }
    // Check relay 5
    if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_5) == Bit_SET) {
        data[1] |= 1 << 3;
    } else {
        data[1] |= 0 << 3;
    }
    // Check relay 6
    if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4) == Bit_SET) {
        data[1] |= 1 << 2;
    } else {
        data[1] |= 0 << 2;
    }
    // Check relay 7
    if (GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_2) == Bit_SET) {
        data[1] |= 1 << 1;
    } else {
        data[1] |= 0 << 1;
    }
    // Check relay 8
    if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_8) == Bit_SET) {
        data[1] |= 1;
    } else {
        data[1] |= 0;
    }
    // Check relay 9
    if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_11) == Bit_SET) {
        data[2] |= 1 << 7;
    } else {
        data[2] |= 0 << 7;
    }
    // Check relay 10
    if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_12) == Bit_SET) {
        data[2] |= 1 << 6;
    } else {
        data[2] |= 0 << 6;
    }
    // Check relay 11
    if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_15) == Bit_SET) {
        data[2] |= 1 << 5;
    } else {
        data[2] |= 0 << 5;
    }
    // Check relay 12
    if (GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_3) == Bit_SET) {
        data[2] |= 1 << 4;
    } else {
        data[2] |= 0 << 4;
    }
}
#if 0
void SwitchRelay(u8 *data)
{
    u8 err = 0;
    // Meter A , 2 outlets
    I2C_SetDeviceAddr(I2C1, 0xF9); // slave addr = 0x7C SID 00
    osDelay(1);

    // outlet 1
    if(curr_state[0] != outlets_state[0])               // outlet state change
    {
        err = SendCommand(2, RELAY_ADDRH, RELAY_ADDRL); // get register
        err = RecvData(2);
        if(outlets_state[0] == '1')                     // relay on
        {
            if(trip[1] > 0)
            {
                I2CTxBuff[0] = I2CRxBuff[0];
                I2CTxBuff[1] = I2CRxBuff[1] & 0xBD;     // disable OCPB_EN INST_IB_EN
                osDelay(5);
                WriteData(1, RELAY_ADDRH, RELAY_ADDRL); // write 2 byrtes register to PL7413
                osDelay(5);
                I2CTxBuff[0] = I2CRxBuff[0];
                I2CTxBuff[1] = I2CRxBuff[1] | 0x42;     // enable OCPB_EN INST_IB_EN
                osDelay(5);
                WriteData(1, RELAY_ADDRH, RELAY_ADDRL); // write 2 byrtes register to PL7413
                osDelay(5);
                trip[1] = 0;
            }
            I2CTxBuff[0] = I2CRxBuff[0] | 0x02;         // turn on relay in register
            curr_state[0] = outlets_state[0];           // mm32 relay state record
        }
        else
        {
            I2CTxBuff[0] = I2CRxBuff[0] & 0xFD;         // turn off relay in register
            curr_state[0] = outlets_state[0];           // mm32 relay state record
        }
        osDelay(10);
        WriteData(0, RELAY_ADDRH, RELAY_ADDRL);         // write 1 byte register to PL7413
        osDelay(890);                                   // delay 890 ms to set next relay
    }

    // outlet 2
    if(curr_state[1] != outlets_state[1])
    {
        err = SendCommand(2, RELAY_ADDRH, RELAY_ADDRL);
        err = RecvData(2);
        if(outlets_state[1] == '1')
        {
            if(trip[2] > 0)
            {
                I2CTxBuff[0] = I2CRxBuff[0];
                I2CTxBuff[1] = I2CRxBuff[1] & 0x6F;     // disable OCPC_EN INST_IC_EN
                osDelay(5);
                WriteData(1, RELAY_ADDRH, RELAY_ADDRL); // write 2 byrtes register to PL7413
                osDelay(5);
                I2CTxBuff[0] = I2CRxBuff[0];
                I2CTxBuff[1] = I2CRxBuff[1] | 0x90;     // enable OCPC_EN INST_IC_EN
                osDelay(5);
                WriteData(1, RELAY_ADDRH, RELAY_ADDRL); // write 2 byrtes register to PL7413
                osDelay(5);
                trip[2] = 0;
            }
            I2CTxBuff[0] = I2CRxBuff[0] | 0x01;
            curr_state[1] = outlets_state[1];
        }
        else
        {
            I2CTxBuff[0] = I2CRxBuff[0] & 0xFE;
            curr_state[1] = outlets_state[1];
        }
        osDelay(10);
        WriteData(0, RELAY_ADDRH, RELAY_ADDRL);
        osDelay(890);                                   // delay 890 ms to set next relay
    }
    err = SendCommand(0, RELAY_ADDRH, RELAY_ADDRL);
    if(err == 1)
            i2c_err_status = i2c_err_status | 0x01;
    RecvData(1);
    if(err == 1)
            i2c_err_status = i2c_err_status | 0x01;
    if((I2CRxBuff[0] & 0x02) == 0x02)
        data[2] = '1';
    if((I2CRxBuff[0] & 0x01) == 0x01)
        data[3] = '1';
    osDelay(10);
    
    
    // Meter B , 3 outlets
    I2C_SetDeviceAddr(I2C1, 0xFD); // slave addr = 0x7E SID 10
    osDelay(1);

    // outlet 3
    if(curr_state[2] != outlets_state[2])
    {
        err = SendCommand(2, RELAY_ADDRH, RELAY_ADDRL);
        err = RecvData(2);
        if(outlets_state[2] == '1')
        {
            if(trip[3] > 0)
            {
                I2CTxBuff[0] = I2CRxBuff[0] & 0xDF;     // disable OCPA_EN
                I2CTxBuff[1] = I2CRxBuff[1] & 0xFB;     // disable INST_IA_EN
                osDelay(5);
                WriteData(1, RELAY_ADDRH, RELAY_ADDRL); // write 2 byrtes register to PL7413
                osDelay(5);
                I2CTxBuff[0] = I2CRxBuff[0] | 0x20;     // enable OCPA_EN
                I2CTxBuff[1] = I2CRxBuff[1] | 0x04;     // enable INST_IA_EN
                osDelay(5);
                WriteData(1, RELAY_ADDRH, RELAY_ADDRL); // write 2 byrtes register to PL7413
                osDelay(5);
                trip[3] = 0;
            }
            I2CTxBuff[0] = I2CRxBuff[0] | 0x04;
            curr_state[2] = outlets_state[2];
        }
        else
        {
            I2CTxBuff[0] = I2CRxBuff[0] & 0xFB;
            curr_state[2] = outlets_state[2];
        }
        osDelay(10);
        WriteData(0, RELAY_ADDRH, RELAY_ADDRL);
        osDelay(890);                                   // delay 890 ms to set next relay
    }
    // outlet 4
    if(curr_state[3] != outlets_state[3])
    {
        err = SendCommand(2, RELAY_ADDRH, RELAY_ADDRL);
        err = RecvData(2);
        if(outlets_state[3] == '1')
        {
            if(trip[4] > 0)
            {
                I2CTxBuff[0] = I2CRxBuff[0];
                I2CTxBuff[1] = I2CRxBuff[1] & 0xBD;     // disable OCPB_EN INST_IB_EN
                osDelay(5);
                WriteData(1, RELAY_ADDRH, RELAY_ADDRL); // write 2 byrtes register to PL7413
                osDelay(5);
                I2CTxBuff[0] = I2CRxBuff[0];
                I2CTxBuff[1] = I2CRxBuff[1] | 0x42;     // enable OCPB_EN INST_IB_EN
                osDelay(5);
                WriteData(1, RELAY_ADDRH, RELAY_ADDRL); // write 2 byrtes register to PL7413
                osDelay(5);
                trip[4] = 0;
            }
            I2CTxBuff[0] = I2CRxBuff[0] | 0x02;
            curr_state[3] = outlets_state[3];
        }
        else
        {
            I2CTxBuff[0] = I2CRxBuff[0] & 0xFD;
            curr_state[3] = outlets_state[3];
        }
        osDelay(10);
        WriteData(0, RELAY_ADDRH, RELAY_ADDRL);
        osDelay(890);                                   // delay 890 ms to set next relay
    }
    // outlet 5
    if(curr_state[4] != outlets_state[4])
    {
        err = SendCommand(2, RELAY_ADDRH, RELAY_ADDRL);
        err = RecvData(2);
        if(outlets_state[4] == '1')
        {
            if(trip[5] > 0)
            {
                I2CTxBuff[0] = I2CRxBuff[0];
                I2CTxBuff[1] = I2CRxBuff[1] & 0x6F;     // disable OCPC_EN INST_IC_EN
                osDelay(5);
                WriteData(1, RELAY_ADDRH, RELAY_ADDRL); // write 2 byrtes register to PL7413
                osDelay(5);
                I2CTxBuff[0] = I2CRxBuff[0];
                I2CTxBuff[1] = I2CRxBuff[1] | 0x90;     // enable OCPC_EN INST_IC_EN
                osDelay(5);
                WriteData(1, RELAY_ADDRH, RELAY_ADDRL); // write 2 byrtes register to PL7413
                osDelay(5);
                trip[5] = 0;
            }
            I2CTxBuff[0] = I2CRxBuff[0] | 0x01;
            curr_state[4] = outlets_state[4];
        }
        else
        {
            I2CTxBuff[0] = I2CRxBuff[0] & 0xFE;
            curr_state[4] = outlets_state[4];
        }
        osDelay(10);
        WriteData(0, RELAY_ADDRH, RELAY_ADDRL);
        osDelay(890);                                   // delay 890 ms to set next relay
    }
    err = SendCommand(0, RELAY_ADDRH, RELAY_ADDRL);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x02;
    err = RecvData(1);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x02;
    if((I2CRxBuff[0] & 0x04) == 0x04)
        data[4] = '1';
    if((I2CRxBuff[0] & 0x02) == 0x02)
        data[5] = '1';
    if((I2CRxBuff[0] & 0x01) == 0x01)
        data[6] = '1';
    osDelay(10);

    // Meter C , 3 outlets
    I2C_SetDeviceAddr(I2C1, 0xFF); // slave addr = 0x7F SID 11
    osDelay(1);

    // outlet 6
    if(curr_state[5] != outlets_state[5])
    {
        err = SendCommand(2, RELAY_ADDRH, RELAY_ADDRL);
        err = RecvData(2);
        if(outlets_state[5] == '1')
        {
            if(trip[6] > 0)
            {
                I2CTxBuff[0] = I2CRxBuff[0] & 0xDF;     // disable OCPA_EN
                I2CTxBuff[1] = I2CRxBuff[1] & 0xFB;     // disable INST_IA_EN
                osDelay(5);
                WriteData(1, RELAY_ADDRH, RELAY_ADDRL); // write 2 byrtes register to PL7413
                osDelay(5);
                I2CTxBuff[0] = I2CRxBuff[0] | 0x20;     // enable OCPA_EN
                I2CTxBuff[1] = I2CRxBuff[1] | 0x04;     // enable INST_IA_EN
                osDelay(5);
                WriteData(1, RELAY_ADDRH, RELAY_ADDRL); // write 2 byrtes register to PL7413
                osDelay(5);
                trip[6] = 0;
            }
            I2CTxBuff[0] = I2CRxBuff[0] | 0x04;
            curr_state[5] = outlets_state[5];
        }
        else
        {
            I2CTxBuff[0] = I2CRxBuff[0] & 0xFB;
            curr_state[5] = outlets_state[5];
        }
        osDelay(10);
        WriteData(0, RELAY_ADDRH, RELAY_ADDRL);
        osDelay(890);                                   // delay 890 ms to set next relay
    }
    // outlet 7
    if(curr_state[6] != outlets_state[6])
    {
        err = SendCommand(2, RELAY_ADDRH, RELAY_ADDRL);
        err = RecvData(2);
        if(outlets_state[6] == '1')
        {
            if(trip[7] > 0)
            {
                I2CTxBuff[0] = I2CRxBuff[0];
                I2CTxBuff[1] = I2CRxBuff[1] & 0xBD;     // disable OCPB_EN INST_IB_EN
                osDelay(5);
                WriteData(1, RELAY_ADDRH, RELAY_ADDRL); // write 2 byrtes register to PL7413
                osDelay(5);
                I2CTxBuff[0] = I2CRxBuff[0];
                I2CTxBuff[1] = I2CRxBuff[1] | 0x42;     // enable OCPB_EN INST_IB_EN
                osDelay(5);
                WriteData(1, RELAY_ADDRH, RELAY_ADDRL); // write 2 byrtes register to PL7413
                osDelay(5);
                trip[7] = 0;
            }
            I2CTxBuff[0] = I2CRxBuff[0] | 0x02;
            curr_state[6] = outlets_state[6];
        }
        else
        {
            I2CTxBuff[0] = I2CRxBuff[0] & 0xFD;
            curr_state[6] = outlets_state[6];
        }
        osDelay(10);
        WriteData(0, RELAY_ADDRH, RELAY_ADDRL);
        osDelay(890);                                   // delay 890 ms to set next relay
    }
    // outlet 8
    if(curr_state[7] != outlets_state[7])
    {
        err = SendCommand(2, RELAY_ADDRH, RELAY_ADDRL);
        err = RecvData(2);
        if(outlets_state[7] == '1')
        {
            if(trip[8] > 0)
            {
                I2CTxBuff[0] = I2CRxBuff[0];
                I2CTxBuff[1] = I2CRxBuff[1] & 0x6F;     // disable OCPC_EN INST_IC_EN
                osDelay(5);
                WriteData(1, RELAY_ADDRH, RELAY_ADDRL); // write 2 byrtes register to PL7413
                osDelay(5);
                I2CTxBuff[0] = I2CRxBuff[0];
                I2CTxBuff[1] = I2CRxBuff[1] | 0x90;     // enable OCPC_EN INST_IC_EN
                osDelay(5);
                WriteData(1, RELAY_ADDRH, RELAY_ADDRL); // write 2 byrtes register to PL7413
                osDelay(5);
                trip[8] = 0;
            }
            I2CTxBuff[0] = I2CRxBuff[0] | 0x01;
            curr_state[7] = outlets_state[7];
        }
        else
        {
            I2CTxBuff[0] = I2CRxBuff[0] & 0xFE;
            curr_state[7] = outlets_state[7];
        }
        osDelay(10);
        WriteData(0, RELAY_ADDRH, RELAY_ADDRL);
        osDelay(890);                                   // delay 890 ms to set next relay
    }
    err = SendCommand(0, RELAY_ADDRH, RELAY_ADDRL);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x04;
    err = RecvData(1);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x04;
    if((I2CRxBuff[0] & 0x04) == 0x04)
        data[7] = '1';
    if((I2CRxBuff[0] & 0x02) == 0x02)
        data[8] = '1';
    if((I2CRxBuff[0] & 0x01) == 0x01)
        data[9] = '1';
}
#endif

void I2C_EmptyTest(void)
{
    I2C_SetDeviceAddr(I2C1, 0xF9); // slave addr = 0x7C SID 00
    osDelay(100);
    SendCommand(0, VRMS_ADDRH, VRMS_ADDRL);
    RecvData(6);
    osDelay(900);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  I2C1 Master test
/// @note   None.
/// @param  None.
/// @retval None.
////////////////////////////////////////////////////////////////////////////////
void I2C_MeterData(void)
{
    u8 err = 0;
    /*********************************************************************
       tx and rx complete
       need 3 times to get meter A, B, C(8 outlet and 1 total data)
    **********************************************************************/

    // 7C OK
    I2C_SetDeviceAddr(I2C1, 0xF9); // slave addr(SID) = 0x7C + R(1)/W(0) = 0111 1100 << 1 + R(1) = 1111 1001
    osDelay(10);
    
    // voltage
    err = DataHandler(VOLTA);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x01;
    osDelay(10);
    // Hz
    err = DataHandler(HZ);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x01;
    osDelay(10);
    // current
    err = DataHandler(CURRA);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x01;
    osDelay(10);
    // watt
    err = DataHandler(WATTA);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x01;
    osDelay(10);
    // kwh
    err = DataHandler(KWHA);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x01;
    osDelay(10);
    // pf
    DataHandler(PFA);
    osDelay(10);
    // TRIP
    /*err = DataHandler(TRIPA);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x01;
    osDelay(10);*/

    //7D OK
    osDelay(10);
    I2C_SetDeviceAddr(I2C1, 0xFB); // slave addr = 0x7D + R(1)/W(0) = 0111 1101 << 1 + R(1) = 1111 1011
    osDelay(10);

    // voltage
    err = DataHandler(VOLTB);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x02;
    osDelay(10);
    // Hz
    err = DataHandler(HZ);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x02;
    osDelay(10);
    // current
    err = DataHandler(CURRB);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x02;
    osDelay(10);
    // watt
    err = DataHandler(WATTB);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x02;
    osDelay(10);
    // kwh
    err = DataHandler(KWHB);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x02;
    osDelay(10);
    // pf
    DataHandler(PFB);
    osDelay(10);
    // TRIP
    /*err = DataHandler(TRIPB);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x02;
    osDelay(10);*/

    // 7E OK
    osDelay(10);
    I2C_SetDeviceAddr(I2C1, 0xFD); // slave addr = 0x7E + R(1)/W(0) = 0111 1110 << 1 + R(1) = 1111 1101
    osDelay(10);

    // voltage
    err = DataHandler(VOLTC);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x04;
    osDelay(10);
    // Hz
    err = DataHandler(HZ);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x04;
    osDelay(10);
    // current
    err = DataHandler(CURRC);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x04;
    osDelay(10);
    // watt
    err = DataHandler(WATTC);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x04;
    osDelay(10);
    // kwh
    err = DataHandler(KWHC);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x04;
    osDelay(10);
    // pf
    DataHandler(PFC);
    osDelay(10);
    // TRIP
    /*err = DataHandler(TRIPC);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x04;
    osDelay(10);*/

    // 7F OK
    osDelay(10);
    I2C_SetDeviceAddr(I2C1, 0xFF); // slave addr = 0x7F + R(1)/W(0) = 0111 1111 << 1 + R(1) = 1111 1111
    osDelay(10);

    // voltage
    err = DataHandler(VOLTD);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x08;
    osDelay(10);
    // Hz
    err = DataHandler(HZ);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x08;
    osDelay(10);    
    // current
    err = DataHandler(CURRD);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x08;
    osDelay(10);
    // watt
    err = DataHandler(WATTD);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x08;
    osDelay(10);
    // kwh
    err = DataHandler(KWHD);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x08;
    osDelay(10);
    // pf
    DataHandler(PFD);
    osDelay(10);
    // TRIP
    /*err = DataHandler(TRIPD);
    if(err == 1)
        i2c_err_status = i2c_err_status | 0x08;*/

    /* calculate total value
    currentA = o_current[0] + o_current[1] + o_current[2] + o_current[3] + o_current[4] + o_current[5] +
               o_current[6] + o_current[7] + o_current[8] + o_current[9] + o_current[10] + o_current[11];

    wattW = o_kw[0] + o_kw[1] + o_kw[2] + o_kw[3] + o_kw[4] + o_kw[5] +
            o_kw[6] + o_kw[7] + o_kw[8] + o_kw[9] + o_kw[10] + o_kw[11];

    if(voltageVA == 0 | currentA == 0) {
        pf = 0;
    }
    else {
        pf = wattW/voltageVA;
        pf = pf/currentA;
    }
    kwh = o_kwh[0] + o_kwh[1] + o_kwh[2] + o_kwh[3] + o_kwh[4] + o_kwh[5] +
          o_kwh[6] + o_kwh[7] + o_kwh[8] + o_kwh[9] + o_kwh[10] + o_kwh[11];*/
}

/// @}

/// @}

/// @}
