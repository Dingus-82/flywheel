double julianDay(int y, int m, int d, int h, int min, int s) {
    if (m <= 2) {
        y -= 1;
        m += 12;
    }

    long A = y / 100;
    long B = 2 - A + (A / 4);

    double dayFraction = (h + min / 60.0 + s / 3600.0) / 24.0;

    return (int)(365.25 * (y + 4716))
         + (int)(30.6001 * (m + 1))
         + d + B - 1524.5 + dayFraction;
}
double sunAzimuthDeg(
    double JD,
    double latitudeDeg,
    double hourUTC,
    double minuteUTC,
    double secondUTC
) {
    // days since J2000
    double n = JD - 2451545.0;

    // mean longitude
    double L = fmod(280.46 + 0.9856474 * n, 360.0);

    // mean anomaly
    double g = fmod(357.528 + 0.9856003 * n, 360.0);
    double gRad = g * DEG_TO_RAD;

    // ecliptic longitude
    double lambda = L + 1.915 * sin(gRad) + 0.020 * sin(2 * gRad);
    double lambdaRad = lambda * DEG_TO_RAD;

    // obliquity of ecliptic
    double epsilon = 23.439 * DEG_TO_RAD;

    // declination
    double delta = asin(sin(epsilon) * sin(lambdaRad));

    // solar time → hour angle (simplified)
    double timeUTC = hourUTC + minuteUTC / 60.0 + secondUTC / 3600.0;
    double H = (timeUTC - 12.0) * 15.0 * DEG_TO_RAD;

    // latitude
    double phi = latitudeDeg * DEG_TO_RAD;

    // azimuth
    double az = atan2(
        sin(H),
        cos(H) * sin(phi) - tan(delta) * cos(phi)
    );

    double azDeg = az * RAD_TO_DEG;

    // convert to compass heading (0–360)
    double heading = fmod((azDeg + 180.0), 360.0);
    if (heading < 0) heading += 360.0;

    return heading;
}
