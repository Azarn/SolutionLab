import datetime
import math


EPOCH_JD = 2451545.0


def get_JD(date):
    month = date.month
    year = date.year
    if month <= 2:
        month += 12
        year -= 1
    return int(365.25 * year) + int(30.6001 * (month + 1)) - 15 + 1720996.5 + (
        date.day + date.hour / 24.0 + date.minute / (24.0 * 60) + date.second / (24.0 * 60 * 60))


def get_solar_coordinates(jd, t):
    # mean anomaly
    M = math.radians((357.52910 + 35999.05030 * t - 0.0001559 * t * t - 0.00000048 * t * t * t) % 360)
    # mean longitude, degree
    L0 = (280.46645 + 36000.76983 * t + 0.0003032 * t * t) % 360

    DL = (1.914600 - 0.004817 * t - 0.000014 * t * t) * math.sin(M) + (
        (0.019993 - 0.000101 * t) * math.sin(2 * M) + 0.000290 * math.sin(3 * M))

    # true longitude, degree
    L = L0 + DL

    return math.radians(L0), math.radians(L)


def get_ra_and_delta(l, jd):
    # obliquity eps of ecliptic:
    eps = math.radians(23.43929111 - 3.563E-7 * jd)
    #print(3.563E-7 * jd)

    x_eclip = math.cos(l)
    y_eclip = math.sin(l)

    x = x_eclip
    y = y_eclip * math.cos(eps)
    z = y_eclip * math.sin(eps)

    delta = math.atan2(z, math.hypot(x, y))
    ra = math.atan2(y, x)
    return delta, ra


def get_sidereal_time_at_lng(lng, l0, hour):
    #theta0 = math.radians((280.46061837 + 360.98564736629 * (jd - EPOCH_JD) + 0.000387933 * t * t - t * t * t / 38710000.0) % 360)

    theta0 = l0 + math.pi + math.radians(hour * 15)
    #print(math.degrees(theta0) % 360, math.degrees(theta1) % 360)

    theta = theta0 + lng
    return theta


def get_altitude_and_azimuth(delta, tau, lat):
    x = math.cos(tau) * math.cos(delta)
    y = math.sin(tau) * math.cos(delta)
    z = math.sin(delta)

    xhor = x * math.sin(lat) - z * math.cos(lat)
    yhor = y
    zhor = x * math.cos(lat) + z * math.sin(lat)

    h = math.atan2(zhor, math.hypot(xhor, yhor))
    az = math.atan2(yhor, xhor) + math.pi
    return h, az


def main():
    #now = datetime.datetime.utcnow()
    now = datetime.datetime(year=2016, month=9, day=27, hour=11, minute=25)
    where = (59.9577927, 30.4167652)
    #where = (50, 10)
    #now = datetime.datetime(year=1991, month=5, day=19, hour=13)
    print("Now (UTC):", now)

    jd = get_JD(now)
    print("JD:", jd)

    t = (jd - EPOCH_JD) / 36525     # Number of Julian centuries since j2000
    print("T:", t)

    l0, l = get_solar_coordinates(jd, t)
    print("L0:", math.degrees(l0))
    print("L:", math.degrees(l))

    delta, ra = get_ra_and_delta(l, jd - EPOCH_JD)
    print("delta:", math.degrees(delta))
    print("ra:", math.degrees(ra))

    theta = get_sidereal_time_at_lng(math.radians(where[1]), l0, now.hour + now.minute / 60 + now.second / 3600)
    print("theta", math.degrees(theta) % 360)

    tau = theta - ra
    print("tau:", math.degrees(tau) % 360)

    h, az = get_altitude_and_azimuth(delta, tau, math.radians(where[0]))
    print("h:", h, math.degrees(h))
    print("az:", az, math.degrees(az))


if __name__ == '__main__':
    main()
