
#ifndef MAP_SERVER_MAP_SERVER_H
#define MAP_SERVER_MAP_SERVER_H

/*
 * Author: Brian Gerkey
 */

#include "nav_msgs/GetMap.h"

/** Map mode
 * % 地图的三种加载·模式。
 * //$ TRINARY模式：占据（Occupied）像素用100表示，自由（Free）像素用0表示，不明（Unknow）像素用-1表示；
   //$ SCALE模式：占据（Occupied）像素用100表示，自由（Free）像素用0表示，不明（Unknow）像素用(0, 100)；
   //$ RAW模式：所有像素用 [0, 255] 表示，直接输出像素值为栅格值。

   //# 接着通过图片信息计算 rgb 平均值 color_avg，根据计算公式 occ = (255 - color_avg) / 255.0；计算每个像素的占用概率 occ。

   //# 在TRINARY 模式下，如果 occ 大于占用概率阈值 occ_th，则当前像素被占用（用100表示），小于 free_th（用0表示）、不确定（用-1表示）。

 *  Default: TRINARY -
 *      value >= occ_th - Occupied (100)
 *      value <= free_th - Free (0)
 *      otherwise - Unknown
 *  SCALE -
 *      alpha < 1.0 - Unknown
 *      value >= occ_th - Occupied (100)
 *      value <= free_th - Free (0)
 *      otherwise - f( (free_th, occ_th) ) = (0, 100)
 *          (linearly map in between values to (0,100)
 *  RAW -
 *      value = value
 */
enum MapMode {TRINARY, SCALE, RAW};

namespace map_server
{

/** Read the image from file and fill out the resp object, for later
 * use when our services are requested.
 *
 * @param resp The map wil be written into here
 * @param fname The image file to read from
 * @param res The resolution of the map (gets stored in resp)
 * @param negate If true, then whiter pixels are occupied, and blacker
 *               pixels are free
 * @param occ_th Threshold above which pixels are occupied
 * @param free_th Threshold below which pixels are free
 * @param origin Triple specifying 2-D pose of lower-left corner of image
 * @param mode Map mode
 * @throws std::runtime_error If the image file can't be loaded
 * */
void loadMapFromFile(nav_msgs::GetMap::Response* resp,
                     const char* fname, double res, bool negate,
                     double occ_th, double free_th, double* origin,
                     MapMode mode=TRINARY);
}

#endif
