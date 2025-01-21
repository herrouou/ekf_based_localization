#ifndef GEO_TRANSFORM_H_
#define GEO_TRANSFORM_H_

#include <cmath>

namespace geo_transform
{
    class GeoTransform
    {
    public:


        GeoTransform(double ref_lat, double ref_lon, double ref_alt);


        void setReference(double ref_lat, double ref_lon, double ref_alt);


        void lla2enu(double lat, double lon, double alt, double &x, double &y, double &z);


        void enu2lla(double x, double y, double z, double &lat, double &lon, double &alt);

    private:
        // 参考点的经纬度和高度(弧度和米)
        double ref_lat_rad_;
        double ref_lon_rad_;
        double ref_alt_;

        // 参考点的ECEF坐标
        double ref_x_;
        double ref_y_;
        double ref_z_;

        // WGS84常量
        const double a_ = 6378137.0;         // 地球半径(长半轴)
        const double f_ = 1.0 / 298.257223563; // 扁率
        const double b_ = a_ * (1 - f_);
        const double e2_ = f_ * (2 - f_);    // 偏心率平方

        // 内部函数: LLA <-> ECEF
        void lla2ecef(double lat, double lon, double alt, double &x, double &y, double &z);
        void ecef2lla(double x, double y, double z, double &lat, double &lon, double &alt);
    };
}

#endif
