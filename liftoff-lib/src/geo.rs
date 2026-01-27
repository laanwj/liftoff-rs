use std::f64::consts::PI;

pub fn gps_from_coord(coord: &[f64; 3], base: (f64, f64)) -> (f64, f64, f64) {
    let x = coord[0]; // Longitude related
    let y = coord[2]; // Latitude related

    let base_lon = base.0;
    let base_lat = base.1;

    let latitude = base_lat + y / 111111.0;
    let longitude = base_lon + x / (111111.0 * (latitude.to_radians().cos()));
    let altitude = coord[1];

    (longitude, latitude, altitude)
}

pub fn coord_from_gps(gps: (f64, f64, f64), base: (f64, f64)) -> [f64; 3] {
    let (lon, lat, alt) = gps;
    let (base_lon, base_lat) = base;

    let y = (lat - base_lat) * 111111.0;
    let x = (lon - base_lon) * 111111.0 * (lat.to_radians().cos());
    let z = y; // Z is North in our local frame logic for now, matching gps_from_coord input [x, alt, y(north)]

    [x, alt, z]
}

pub fn quat2heading(q0: f64, q1: f64, q2: f64, q3: f64) -> f64 {
    let y = 2.0 * ((q2 * q0) + (q3 * q1));
    let x = q3.powi(2) + q2.powi(2) - q0.powi(2) - q1.powi(2);
    y.atan2(x)
}

pub fn quat2eulers(qx: f64, qy: f64, qz: f64, qw: f64) -> (f64, f64, f64) {
    // swap (liftoff y-up to imu coordinate system z-up)
    // (qx, qy, qz, qw) = (qx, qz, qy, -qw)
    let (qx, qy, qz, qw) = (qx, qz, qy, -qw);

    let m00 = 1.0 - 2.0 * qy * qy - 2.0 * qz * qz;
    let m10 = 2.0 * (qx * qy + qw * qz);
    let m20 = 2.0 * (qx * qz - qw * qy);
    let m21 = 2.0 * (qy * qz + qw * qx);
    let m22 = 1.0 - 2.0 * qx * qx - 2.0 * qy * qy;

    let roll = m21.atan2(m22);
    let pitch = (0.5 * PI) - (-m20).acos();
    let yaw = -m10.atan2(m00);

    (roll, pitch, yaw)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_quat2heading() {
        // N  0.000, 0.000, 0.000, 1.000     0
        assert_eq!(quat2heading(0.0, 0.0, 0.0, 1.0).to_degrees().round(), 0.0);
        // W  0.000,-0.707, 0.000, 0.707   -90
        assert_eq!(
            quat2heading(0.0, -0.70710678, 0.0, 0.70710678)
                .to_degrees()
                .round(),
            -90.0
        );
        // S  0.000,-1.000, 0.000, 0.000   180
        assert_eq!(
            quat2heading(0.0, -1.0, 0.0, 0.0).to_degrees().round(),
            180.0
        );
        // E  0.000,-0.707, 0.000,-0.707    90
        assert_eq!(
            quat2heading(0.0, -0.70710678, 0.0, -0.70710678)
                .to_degrees()
                .round(),
            90.0
        );
    }

    #[test]
    fn test_gps_from_coord() {
        let base = (10.0, 50.0); // Lon, Lat
        let coord = [100.0, 100.0, 100.0]; // x (lon), alt, y (lat)
        // x is roughly east, y is roughly north.
        let (lon, lat, alt) = gps_from_coord(&coord, base);

        assert_eq!(alt, 100.0);
        assert!(lat > base.1); // Should be north of base
        assert!(lon > base.0); // Should be east of base

        // Rough check: 1 degree lat is ~111km, 100m is ~0.0009 degrees
        assert!((lat - base.1).abs() < 0.01);
        assert!((lon - base.0).abs() < 0.01);
    }

    #[test]
    fn test_quat2eulers_identity() {
        let (roll, pitch, yaw) = quat2eulers(0.0, 0.0, 0.0, 1.0);
        assert_eq!(roll, 0.0);
        assert_eq!(pitch, 0.0);
        assert_eq!(yaw, 0.0);
    }
}
