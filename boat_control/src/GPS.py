import math

def GPS_to_BD(lat,lon):
	lat,lon=gps84_To_Gcj02(lat, lon)
	lat,lon=gcj02_To_Bd09(lat, lon)
	return lat,lon


def BD_to_GPS(lat,lon):
	lat,lon=Bd09_To_gcj02(lat, lon)
	lat,lon=gcj02_To_gps84(lat, lon)
	return lat,lon


def transformLat(x, y):
    pi = 3.1415926535897932384626
    a = 6378245.0
    ee = 0.00669342162296594323
    x_Pi = 3.14159265358979324 * 3000.0 / 180.0
    ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y+ 0.2 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * pi) + 20.0 * math.sin(2.0 * x * pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(y * pi) + 40.0 * math.sin(y / 3.0 * pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(y / 12.0 * pi) + 320 * math.sin(y * pi / 30.0)) * 2.0 / 3.0
    return ret


def transformLon(x, y):
    pi = 3.1415926535897932384626
    a = 6378245.0
    ee = 0.00669342162296594323
    x_Pi = 3.14159265358979324 * 3000.0 / 180.0
    ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1* math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * pi) + 20.0 * math.sin(2.0 * x * pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(x * pi) + 40.0 * math.sin(x / 3.0 * pi)) * 2.0 / 3.0
    ret += (150.0 *math.sin(x / 12.0 * pi) + 300.0 * math.sin(x / 30.0* pi)) * 2.0 / 3.0
    return ret


def gps84_To_Gcj02(lat, lon):
    pi = 3.1415926535897932384626
    a = 6378245.0
    ee = 0.00669342162296594323
    x_Pi = 3.14159265358979324 * 3000.0 / 180.0
    dLat = transformLat(lon - 105.0, lat - 35.0)
    dLon = transformLon(lon - 105.0, lat - 35.0)
    radLat = lat / 180.0 * pi
    magic = math.sin(radLat)
    magic = 1 - ee * magic * magic
    sqrtMagic = math.sqrt(magic)
    dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * pi)
    dLon = (dLon * 180.0) / (a / sqrtMagic * math.cos(radLat) * pi)
    lat = lat + dLat
    lon = lon + dLon
    return lat,lon

def gcj02_To_Bd09(gg_lat, gg_lon):
    pi = 3.1415926535897932384626
    a = 6378245.0
    ee = 0.00669342162296594323
    x_Pi = 3.14159265358979324 * 3000.0 / 180.0
    x = gg_lon
    y = gg_lat
    z = math.sqrt(x * x + y * y) + 0.00002 * math.sin(y * x_Pi)
    theta = math.atan2(y, x) + 0.000003 *math.cos(x * x_Pi)
    gg_lon = z * math.cos(theta) + 0.0065
    gg_lat = z * math.sin(theta) + 0.006
    return gg_lat,gg_lon

def Bd09_To_gcj02(gg_lat, gg_lon):
    pi = 3.1415926535897932384626
    a = 6378245.0
    ee = 0.00669342162296594323
    x_Pi = 3.14159265358979324 * 3000.0 / 180.0
    x = gg_lon - 0.0065
    y = gg_lat - 0.006
    z = math.sqrt(x * x + y * y) - 0.00002 * math.sin(y * x_Pi)
    theta = math.atan2(y, x) - 0.000003 *math.cos(x * x_Pi)
    gg_lon = z * math.cos(theta)
    gg_lat = z * math.sin(theta)
    return gg_lat,gg_lon

def gcj02_To_gps84(lat, lon):
    pi = 3.1415926535897932384626
    a = 6378245.0
    ee = 0.00669342162296594323
    x_Pi = 3.14159265358979324 * 3000.0 / 180.0
    dLat = transformLat(lon - 105.0, lat - 35.0)
    dLon = transformLon(lon - 105.0, lat - 35.0)
    radLat = lat / 180.0 * pi
    magic = math.sin(radLat)
    magic = 1 - ee * magic * magic
    sqrtMagic = math.sqrt(magic)
    dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * pi)
    dLon = (dLon * 180.0) / (a / sqrtMagic * math.cos(radLat) * pi)
    lat -= dLat
    lon -= dLon
    return lat,lon