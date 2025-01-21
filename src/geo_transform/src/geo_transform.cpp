#include "geo_transform/geo_transform.h"
#include <cmath>
#include <stdexcept>

namespace geo_transform
{

GeoTransform::GeoTransform(double ref_lat, double ref_lon, double ref_alt)
{
    setReference(ref_lat, ref_lon, ref_alt);
}

void GeoTransform::setReference(double ref_lat, double ref_lon, double ref_alt)
{
    ref_lat_rad_ = ref_lat * M_PI / 180.0;
    ref_lon_rad_ = ref_lon * M_PI / 180.0;
    ref_alt_ = ref_alt;

    // ECEF
    lla2ecef(ref_lat_rad_, ref_lon_rad_, ref_alt_, ref_x_, ref_y_, ref_z_);
}

void GeoTransform::lla2ecef(double lat, double lon, double alt, double &x, double &y, double &z)
{
    // lat, lon -> rad
    double sin_lat = std::sin(lat);
    double cos_lat = std::cos(lat);
    double sin_lon = std::sin(lon);
    double cos_lon = std::cos(lon);

    double N = a_ / std::sqrt(1 - e2_ * sin_lat * sin_lat);

    x = (N + alt) * cos_lat * cos_lon;
    y = (N + alt) * cos_lat * sin_lon;
    z = (N * (1 - e2_) + alt) * sin_lat;
}

void GeoTransform::ecef2lla(double x, double y, double z, double &lat, double &lon, double &alt)
{
    // ECEF -> LLA（refer to WGS84）
    // iteration ro get lat

    double eps = 1e-12;
    double p = std::sqrt(x * x + y * y);
    double theta = std::atan2(z * a_, p * b_);

    double sin_theta = std::sin(theta);
    double cos_theta = std::cos(theta);

    // innverse lat
    lat = std::atan2(z + e2_ * b_ * sin_theta * sin_theta * sin_theta,
                     p - e2_ * a_ * cos_theta * cos_theta * cos_theta);

    double sin_lat = std::sin(lat);
    double N = a_ / std::sqrt(1 - e2_ * sin_lat * sin_lat);

    alt = p / std::cos(lat) - N;
    lon = std::atan2(y, x);

    // to degree
    lat = lat * 180.0 / M_PI;
    lon = lon * 180.0 / M_PI;
}

void GeoTransform::lla2enu(double lat, double lon, double alt, double &x, double &y, double &z)
{
    // lat, lon to rad
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;

    
    double X, Y, Z;
    lla2ecef(lat_rad, lon_rad, alt, X, Y, Z);

   
    double dx = X - ref_x_;
    double dy = Y - ref_y_;
    double dz = Z - ref_z_;

    // ECEF to ENU's rotation matrix
    double sin_lat0 = std::sin(ref_lat_rad_);
    double cos_lat0 = std::cos(ref_lat_rad_);
    double sin_lon0 = std::sin(ref_lon_rad_);
    double cos_lon0 = std::cos(ref_lon_rad_);

    double t[3][3] = {
        { -sin_lon0,             cos_lon0,             0 },
        { -sin_lat0*cos_lon0,  -sin_lat0*sin_lon0,   cos_lat0 },
        {  cos_lat0*cos_lon0,   cos_lat0*sin_lon0,   sin_lat0 }
    };

    x = t[0][0]*dx + t[0][1]*dy + t[0][2]*dz;
    y = t[1][0]*dx + t[1][1]*dy + t[1][2]*dz;
    z = t[2][0]*dx + t[2][1]*dy + t[2][2]*dz;
}


void GeoTransform::enu2lla(double x, double y, double z, double &lat, double &lon, double &alt)
{
    // ENU to ECEF
    double sin_lat0 = std::sin(ref_lat_rad_);
    double cos_lat0 = std::cos(ref_lat_rad_);
    double sin_lon0 = std::sin(ref_lon_rad_);
    double cos_lon0 = std::cos(ref_lon_rad_);

    double t_inv[3][3] = {
        { -sin_lon0,                        -sin_lat0*cos_lon0,             cos_lat0*cos_lon0 },
        {  cos_lon0,                        -sin_lat0*sin_lon0,             cos_lat0*sin_lon0 },
        {  0,                                cos_lat0,                       sin_lat0         }
    };

    double dx = t_inv[0][0]*x + t_inv[0][1]*y + t_inv[0][2]*z;
    double dy = t_inv[1][0]*x + t_inv[1][1]*y + t_inv[1][2]*z;
    double dz = t_inv[2][0]*x + t_inv[2][1]*y + t_inv[2][2]*z;

    double X = dx + ref_x_;
    double Y = dy + ref_y_;
    double Z = dz + ref_z_;

    // ECEF to LLA
    ecef2lla(X, Y, Z, lat, lon, alt);
}

} // namespace geo_transform
