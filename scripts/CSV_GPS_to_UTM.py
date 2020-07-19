import numpy as np
import pandas as pd
import glob

class CoordConverter:
    sm_a = 6.37814e+06
    sm_b = 6.35675e+06
    sm_EccSquared = 0.00669438
    UTMScaleFactor = 0.9996

    def rotate_xy(cls, x, y, theta):
        costheta = np.cos(theta)
        sintheta = np.sin(theta)
        RotateArray = np.array([
            [
                costheta,
                -sintheta],
            [
                sintheta,
                costheta]])
        a = np.matmul(RotateArray, np.array([
            [
                x],
            [
                y]]))
        return (a[(0, 0)], a[(1, 0)])

    rotate_xy = classmethod(rotate_xy)

    def get_curvature(cls, x1, y1, theta1, x2, y2, theta2):
        norm = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return (theta2 - theta1) / norm

    get_curvature = classmethod(get_curvature)

    def get_distance_UTM(cls, x1, y1, x2, y2):
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    get_distance_UTM = classmethod(get_distance_UTM)

    def get_distance_LatLon(cls, lat1, lon1, lat2, lon2):
        (x1, y1) = cls.LatLonToUTMXY(lat1, lon1)
        (x2, y2) = cls.LatLonToUTMXY(lat2, lon2)
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    get_distance_LatLon = classmethod(get_distance_LatLon)

    def get_azimuth(cls, x, y):
        norm = np.sqrt(x ** 2 + y ** 2)
        theta = np.arccos(y / norm) * 180 / np.pi
        if x < 0:
            theta = 360 - theta
        return theta

    get_azimuth = classmethod(get_azimuth)

    # x0,y0:当前点 x1,y1:检索点
    def get_relative_coord(cls, x0, y0, theta0, x1, y1, theta1):
        x = x1 - x0
        y = y1 - y0
        theta = 90 - theta0
        (x, y) = cls.rotate_xy(x, y, cls.DegToRad(-theta))
        azimuth = theta1 - theta0
        if azimuth > 180:
            azimuth -= 360
        elif azimuth < -180:
            azimuth += 360
        return (x, y, azimuth)

    get_relative_coord = classmethod(get_relative_coord)

    def get_global_coord(cls, x0, y0, theta0, x1, y1, theta1):
        (x2, y2, azimuth2) = cls.get_relative_coord(x0, y0, theta0, 0, 0, 90)
        return cls.get_relative_coord(x2, y2, azimuth2, x1, y1, theta1)

    get_global_coord = classmethod(get_global_coord)

    def BezierInterp(cls, x0, y0, x1, y1, x2, y2, distance):
        ax1 = (x0 - 2 * x1) + x2
        bx1 = 2 * (x1 - x0)
        cx1 = x0
        ay1 = (y0 - 2 * y1) + y2
        by1 = 2 * (y1 - y0)
        cy1 = y0
        p_list = []
        t = 0
        dotx_t = 2 * ax1 * t + bx1
        doty_t = 2 * ay1 * t + by1
        theta = cls.get_azimuth(dotx_t, doty_t)
        while t < 1:
            dt = 0
            dx = distance * np.sin((theta / 180) * np.pi)
            dy = distance * np.cos((theta / 180) * np.pi)
            if dotx_t == 0:
                dt = dy / doty_t
            else:
                dt = dx / dotx_t
            t = t + dt
            if t > 1:
                break
            x = ax1 * t * t + bx1 * t + cx1
            y = ay1 * t * t + by1 * t + cy1
            dotx_t = 2 * ax1 * t + bx1
            doty_t = 2 * ay1 * t + by1
            theta = cls.get_azimuth(dotx_t, doty_t)
            p_list.append(np.array([
                x,
                y,
                theta]))
        return p_list

    BezierInterp = classmethod(BezierInterp)

    def ComputeCubic(cls, x, y, azimuth):
        dy = np.tan(cls.DegToRad(90 - azimuth))
        a = (dy * x - 2 * y) / x ** 3
        b = (3 * y - dy * x) / x ** 2
        return (a, b)

    ComputeCubic = classmethod(ComputeCubic)


    def RoatateWGS84Vector(cls, lon0, lat0, azimuth0, lon1, lat1, azimuth1, angle):
        (x0, y0) = cls.LatLonToUTMXY(lat0, lon0)
        (x1, y1) = cls.LatLonToUTMXY(lat1, lon1)
        (x1_local, y1_local, azimuth_local) = cls.get_relative_coord(x0, y0, azimuth0, x1, y1, azimuth1)
        zone = cls.get_zone(lon0)
        (x1_r_local, y1_r_local) = cls.rotate_xy(x1_local, y1_local, cls.DegToRad(angle))
        azimuth_local -= angle
        if azimuth_local < 0:
            azimuth_local += 360
        elif azimuth_local > 360:
            azimuth_local -= 360
        (xr, yr, azimuthr) = cls.get_global_coord(x0, y0, azimuth0, x1_r_local, y1_r_local, azimuth_local)
        (lat, lon) = cls.UTMXYToLatLon(xr, yr, zone, False)
        return (lat, lon, azimuthr)

    RoatateWGS84Vector = classmethod(RoatateWGS84Vector)

    def ReverseWGS84Vector(self, WgsV1):
        WgsV1.azimuth = 360 - WgsV1.azimuth
        return WgsV1

    def MoveLeftWGS(cls, lat, lon, azimuth, Distance):
        MoveAngle = azimuth - 90
        if MoveAngle < 0:
            MoveAngle += 360
        (lat, lon) = cls.MovePointWgsCoor(lat, lon, MoveAngle, Distance)
        return (lat, lon, azimuth)

    MoveLeftWGS = classmethod(MoveLeftWGS)

    def ArcLengthOfMeridian(cls, phi):
        n = (cls.sm_a - cls.sm_b) / (cls.sm_a + cls.sm_b)
        alpha = ((cls.sm_a + cls.sm_b) / 2) * (1 + np.power(n, 2) / 4 + np.power(n, 4) / 64)
        beta = -3 * n / 2 + 9 * np.power(n, 3) / 16 + -3 * np.power(n, 5) / 32
        gamma = 15 * np.power(n, 2) / 16 + -15 * np.power(n, 4) / 32
        delta = -35 * np.power(n, 3) / 48 + 105 * np.power(n, 5) / 256
        epsilon = 315 * np.power(n, 4) / 512
        result = alpha * (
                    phi + beta * np.sin(2 * phi) + gamma * np.sin(4 * phi) + delta * np.sin(6 * phi) + epsilon * np.sin(
                8 * phi))
        return result

    ArcLengthOfMeridian = classmethod(ArcLengthOfMeridian)

    def MapLatLonToXY(cls, phi, lambda1, lambda0):
        ep2 = (np.power(cls.sm_a, 2) - np.power(cls.sm_b, 2)) / np.power(cls.sm_b, 2)
        nu2 = ep2 * np.power(np.cos(phi), 2)
        N = np.power(cls.sm_a, 2) / cls.sm_b * np.sqrt(1 + nu2)
        t = np.tan(phi)
        t2 = t * t
        tmp = t2 * t2 * t2 - np.power(t, 6)
        l = lambda1 - lambda0
        coef13 = (1 - t2) + nu2
        coef14 = (5 - t2) + 9 * nu2 + 4 * nu2 * nu2
        coef15 = (5 - 18 * t2) + t2 * t2 + 14 * nu2 - 58 * t2 * nu2
        coef16 = (61 - 58 * t2) + t2 * t2 + 270 * nu2 - 330 * t2 * nu2
        coef17 = (61 - 479 * t2) + 179 * t2 * t2 - t2 * t2 * t2
        coef18 = (1385 - 3111 * t2) + 543 * t2 * t2 - t2 * t2 * t2
        x = N * np.cos(phi) * l + (N / 6) * np.power(np.cos(phi), 3) * coef13 * np.power(l, 3) + (N / 120) * np.power(
            np.cos(phi), 5) * coef15 * np.power(l, 5) + (N / 5040) * np.power(np.cos(phi), 7) * coef17 * np.power(l, 7)
        y = cls.ArcLengthOfMeridian(phi) + (t / 2) * N * np.power(np.cos(phi), 2) * np.power(l, 2) + (
                    t / 24) * N * np.power(np.cos(phi), 4) * coef14 * np.power(l, 4) + (t / 720) * N * np.power(
            np.cos(phi), 6) * coef16 * np.power(l, 6) + (t / 40320) * N * np.power(np.cos(phi), 8) * coef18 * np.power(
            l, 8)
        return (x, y)

    MapLatLonToXY = classmethod(MapLatLonToXY)

    def DegToRad(cls, deg):
        return (deg / 180) * np.pi

    DegToRad = classmethod(DegToRad)

    def RadToDeg(cls, rad):
        return (rad / np.pi) * 180

    RadToDeg = classmethod(RadToDeg)

    def UTMCentralMeridian(cls, zone):
        return cls.DegToRad(-183 + zone * 6)

    UTMCentralMeridian = classmethod(UTMCentralMeridian)

    def get_zone(cls, lon):
        zone = int(np.floor((lon + 180) / 6) + 1)
        return zone

    get_zone = classmethod(get_zone)

    def LatLonToUTMXY(cls, lat, lon):
        zone = int(np.floor((lon + 180) / 6) + 1)
        (x, y) = cls.MapLatLonToXY(cls.DegToRad(lat), cls.DegToRad(lon), cls.UTMCentralMeridian(zone))
        x = x * cls.UTMScaleFactor + 500000
        y = y * cls.UTMScaleFactor
        if y < 0:
            y += 1e+07
        return (x, y)

    LatLonToUTMXY = classmethod(LatLonToUTMXY)

    def FootpointLatitude(cls, y):
        n = (cls.sm_a - cls.sm_b) / (cls.sm_a + cls.sm_b)
        alpha_ = ((cls.sm_a + cls.sm_b) / 2) * (1 + np.power(n, 2) / 4 + np.power(n, 4) / 64)
        y_ = y / alpha_
        beta_ = 3 * n / 2 + -27 * np.power(n, 3) / 32 + 269 * np.power(n, 5) / 512
        gamma_ = 21 * np.power(n, 2) / 16 + -55 * np.power(n, 4) / 32
        delta_ = 151 * np.power(n, 3) / 96 + -417 * np.power(n, 5) / 128
        epsilon_ = 1097 * np.power(n, 4) / 512
        result = y_ + beta_ * np.sin(2 * y_) + gamma_ * np.sin(4 * y_) + delta_ * np.sin(6 * y_) + epsilon_ * np.sin(
            8 * y_)
        return result

    FootpointLatitude = classmethod(FootpointLatitude)

    def MapXYToLatLon(cls, x, y, lambda0):
        phif = cls.FootpointLatitude(y)
        ep2 = (np.power(cls.sm_a, 2) - np.power(cls.sm_b, 2)) / np.power(cls.sm_b, 2)
        cf = np.cos(phif)
        nuf2 = ep2 * np.power(cf, 2)
        Nf = np.power(cls.sm_a, 2) / cls.sm_b * np.sqrt(1 + nuf2)
        Nfpow = Nf
        tf = np.tan(phif)
        tf2 = tf * tf
        tf4 = tf2 * tf2
        x1frac = 1 / Nfpow * cf
        Nfpow *= Nf
        x2frac = tf / 2 * Nfpow
        Nfpow *= Nf
        x3frac = 1 / 6 * Nfpow * cf
        Nfpow *= Nf
        x4frac = tf / 24 * Nfpow
        Nfpow *= Nf
        x5frac = 1 / 120 * Nfpow * cf
        Nfpow *= Nf
        x6frac = tf / 720 * Nfpow
        Nfpow *= Nf
        x7frac = 1 / 5040 * Nfpow * cf
        Nfpow *= Nf
        x8frac = tf / 40320 * Nfpow
        x2poly = -1 - nuf2
        x3poly = -1 - 2 * tf2 - nuf2
        x4poly = 5 + 3 * tf2 + 6 * nuf2 - 6 * tf2 * nuf2 - 3 * nuf2 * nuf2 - 9 * tf2 * nuf2 * nuf2
        x5poly = 5 + 28 * tf2 + 24 * tf4 + 6 * nuf2 + 8 * tf2 * nuf2
        x6poly = (-61 - 90 * tf2 - 45 * tf4 - 107 * nuf2) + 162 * tf2 * nuf2
        x7poly = -61 - 662 * tf2 - 1320 * tf4 - 720 * tf4 * tf2
        x8poly = 1385 + 3633 * tf2 + 4095 * tf4 + 1575 * tf4 * tf2
        lat = cls.RadToDeg(
            phif + x2frac * x2poly * x * x + x4frac * x4poly * np.power(x, 4) + x6frac * x6poly * np.power(x,
                                                                                                           6) + x8frac * x8poly * np.power(
                x, 8))
        lon = cls.RadToDeg(lambda0 + x1frac * x + x3frac * x3poly * np.power(x, 3) + x5frac * x5poly * np.power(x,
                                                                                                                5) + x7frac * x7poly * np.power(
            x, 7))
        return (lat, lon)

    MapXYToLatLon = classmethod(MapXYToLatLon)

    def UTMXYToLatLon(cls, x, y, zone, southhemi=False):
        x -= 500000
        x /= cls.UTMScaleFactor
        if southhemi:
            y -= 1e+07
        y /= cls.UTMScaleFactor
        cmeridian = cls.UTMCentralMeridian(zone)
        return cls.MapXYToLatLon(x, y, cmeridian)

    UTMXYToLatLon = classmethod(UTMXYToLatLon)

    def move_point_xy(cls, x, y, azimuth, distance):
        angle_rad = cls.DegToRad(azimuth)
        x1 = x + distance * np.sin(angle_rad)
        y1 = y + distance * np.cos(angle_rad)
        return (x1, y1)

    move_point_xy = classmethod(move_point_xy)

    def MovePointWgsCoor(cls, lat, lon, angle, distance):
        angleRad = cls.DegToRad(angle)
        (x, y) = cls.LatLonToUTMXY(lat, lon)
        x += distance * np.sin(angleRad)
        y += distance * np.cos(angleRad)
        zone = cls.get_zone(lon)
        (lat, lon) = cls.UTMXYToLatLon(x, y, zone, False)
        return (lat, lon)

if __name__=='__main__':
    GPS_csv=glob.glob(r"./GPS/*.csv")
    for GPS_name in GPS_csv:
        GPS=pd.read_csv(GPS_name)
        print(GPS_name)
        x=GPS_name.rfind('/')+1
        y=GPS_name.rfind('.')
        UTM=GPS.copy()
        for index,row in GPS.iterrows():
            UTM.iloc[[index],[1]],UTM.iloc[[index],[2]]=CoordConverter.LatLonToUTMXY(row['lat'],row['lon'])
        UTM=UTM.rename(columns={'lon':'x','lat':'y'})
        print(type(UTM))
        UTM.to_csv('./UTM/'+GPS_name[x:y]+'_UTM.csv',index=0)
