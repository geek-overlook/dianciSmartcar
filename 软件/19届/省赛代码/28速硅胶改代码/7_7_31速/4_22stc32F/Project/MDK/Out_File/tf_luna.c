#include "headfile.h"


	TF Lidar1;//tof测距

/**
 * @brief     read luna dist data.
 * @param[in] point Lidar para struct
 * @return    return statue
 * @note      
 */

int LunaReadDist(TF* lidar)
{
    unsigned char buf[2];
		int ret = I2C_RecvBytes(lidar->Address, REG_ADDR_DIST, &buf, sizeof (buf),1000);
    if(0 == ret)
    {
        lidar->distance = buf[0] + (buf[1] << 8);
    }
    return ret;
}
/**
 * @brief     read luna amp data.
 * @param[in] point Lidar para struct
 * @return    return statue
 * @note      
 */
int LunaReadAmp(TF* lidar)
{
    unsigned char buf[2];
		int ret = I2C_RecvBytes(lidar->Address, 0x02, &buf, sizeof (buf),1000);
    if(0 == ret)
    {
        lidar->strength = buf[0] + (buf[1] << 8);
    }
    return ret;
}
/**
 * @brief     read luna temp data.
 * @param[in] point Lidar para struct
 * @return    return statue
 * @note      
 */
int LunaReadTemp(TF* lidar)
{
    unsigned char buf[2];
		int ret = I2C_RecvBytes(lidar->Address, REG_ADDR_TEMP, &buf, sizeof (buf),1000);
    if(0 == ret)
    {
        lidar->temp = buf[0] + (buf[1] << 8);
    }
    return ret;
}









