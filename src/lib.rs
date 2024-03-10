extern crate glm;
extern crate serde;

use serde::{Deserialize, Serialize};
use std::ops;

#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct V3 {
    x: f32,
    y: f32,
    z: f32,
}

impl V3 {
    pub fn new(x: f32, y: f32, z: f32) -> V3 {
        V3 { x, y, z }
    }

    pub fn with_value(value: f32) -> V3 {
        V3::new(value, value, value)
    }

    pub fn length(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn normalize(&mut self) -> V3 {
        let length = self.length();
        self.x /= length;
        self.y /= length;
        self.z /= length;
        *self
    }

    pub fn get_normalized(&self) -> V3 {
        let length = self.length();
        V3::new(self.x / length, self.y / length, self.z / length)
    }
}

impl ops::Add<V3> for V3 {
    type Output = V3;

    fn add(self, rhs: V3) -> V3 {
        V3::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl ops::AddAssign<V3> for V3 {
    fn add_assign(&mut self, rhs: V3) {
        *self = V3::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z);
    }
}

impl ops::Sub<V3> for V3 {
    type Output = V3;

    fn sub(self, rhs: V3) -> V3 {
        V3::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl ops::SubAssign<V3> for V3 {
    fn sub_assign(&mut self, rhs: V3) {
        *self = V3::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z);
    }
}

impl ops::Mul<f32> for V3 {
    type Output = V3;

    fn mul(self, rhs: f32) -> V3 {
        V3::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl ops::MulAssign<f32> for V3 {
    fn mul_assign(&mut self, rhs: f32) {
        *self = V3::new(self.x * rhs, self.y * rhs, self.z * rhs);
    }
}

impl ops::Div<f32> for V3 {
    type Output = V3;

    fn div(self, rhs: f32) -> V3 {
        V3::new(self.x / rhs, self.y / rhs, self.z / rhs)
    }
}

impl ops::DivAssign<f32> for V3 {
    fn div_assign(&mut self, rhs: f32) {
        *self = V3::new(self.x / rhs, self.y / rhs, self.z / rhs);
    }
}

impl ops::Neg for V3 {
    type Output = V3;

    fn neg(self) -> V3 {
        V3::new(-self.x, -self.y, -self.z)
    }
}

impl ops::Mul<V3> for V3 {
    type Output = f32;

    fn mul(self, rhs: V3) -> f32 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }
}

impl ops::BitXor<V3> for V3 {
    type Output = V3;

    fn bitxor(self, rhs: V3) -> V3 {
        V3::new(
            self.y * rhs.z - self.z * rhs.y,
            self.z * rhs.x - self.x * rhs.z,
            self.x * rhs.y - self.y * rhs.x,
        )
    }
}

impl ops::BitXorAssign<V3> for V3 {
    fn bitxor_assign(&mut self, rhs: V3) {
        *self = V3::new(
            self.y * rhs.z - self.z * rhs.y,
            self.z * rhs.x - self.x * rhs.z,
            self.x * rhs.y - self.y * rhs.x,
        );
    }
}

fn lerp(a: V3, b: V3, t: f32) -> V3 {
    a * (1.0 - t) + b * t
}

// TODO(Jovan): Implement
#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct Quat {
    x: f32,
    y: f32,
    z: f32,
    w: f32,
}

#[derive(Copy, Clone, Debug, PartialEq, Default, Serialize, Deserialize)]
pub struct M44 {
    m: [[f32; 4]; 4],
}

impl M44 {
    pub fn new(m: [[f32; 4]; 4]) -> M44 {
        M44 { m }
    }

    pub fn identity() -> M44 {
        M44 {
            m: [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    pub fn with_value(value: f32) -> M44 {
        M44 {
            m: [
                [value, 0.0, 0.0, 0.0],
                [0.0, value, 0.0, 0.0],
                [0.0, 0.0, value, 0.0],
                [0.0, 0.0, 0.0, value],
            ],
        }
    }

    pub fn det(&self) -> f32 {
        self.m[0][0] * (self.m[1][1] * self.m[2][2] - self.m[1][2] * self.m[2][1])
            - self.m[0][1] * (self.m[1][0] * self.m[2][2] - self.m[1][2] * self.m[2][0])
            + self.m[0][2] * (self.m[1][0] * self.m[2][1] - self.m[1][1] * self.m[2][0])
    }

    pub fn transpose(&self) -> M44 {
        M44 {
            m: [
                [self.m[0][0], self.m[1][0], self.m[2][0], self.m[3][0]],
                [self.m[0][1], self.m[1][1], self.m[2][1], self.m[3][1]],
                [self.m[0][2], self.m[1][2], self.m[2][2], self.m[3][2]],
                [self.m[0][3], self.m[1][3], self.m[2][3], self.m[3][3]],
            ],
        }
    }

    pub fn translate(&self, v: V3) -> M44 {
        let mut m = self.m;
        m[0][3] += v.x;
        m[1][3] += v.y;
        m[2][3] += v.z;
        M44 { m }
    }

    pub fn scale(&self, v: V3) -> M44 {
        let mut m = self.m;
        m[0][0] *= v.x;
        m[1][1] *= v.y;
        m[2][2] *= v.z;
        M44 { m }
    }
}

impl ops::Mul<M44> for M44 {
    type Output = M44;

    fn mul(self, rhs: M44) -> M44 {
        let mut m = [[0.0; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                m[i][j] = self.m[i][0] * rhs.m[0][j]
                    + self.m[i][1] * rhs.m[1][j]
                    + self.m[i][2] * rhs.m[2][j]
                    + self.m[i][3] * rhs.m[3][j];
            }
        }
        M44 { m }
    }
}

impl ops::MulAssign<M44> for M44 {
    fn mul_assign(&mut self, rhs: M44) {
        let mut m = [[0.0; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                m[i][j] = self.m[i][0] * rhs.m[0][j]
                    + self.m[i][1] * rhs.m[1][j]
                    + self.m[i][2] * rhs.m[2][j]
                    + self.m[i][3] * rhs.m[3][j];
            }
        }
        self.m = m;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_v3_new() {
        let a = V3::new(1.0, 2.0, 3.0);
        assert_eq!(a.x, 1.0);
        assert_eq!(a.y, 2.0);
        assert_eq!(a.z, 3.0);
    }

    #[test]
    fn test_v3_with_value() {
        let a = V3::with_value(1.0);
        assert_eq!(a.x, 1.0);
        assert_eq!(a.y, 1.0);
        assert_eq!(a.z, 1.0);
    }

    #[test]
    fn test_v3_is_equal() {
        let a = V3::new(1.0, 2.0, 3.0);
        let b = V3::new(1.0, 2.0, 3.0);
        assert_eq!(a, b);
    }

    #[test]
    fn test_v3_is_not_equal() {
        let a = V3::new(1.0, 2.0, 3.0);
        let b = V3::new(1.0, 2.0, 4.0);
        assert_ne!(a, b);
    }

    #[test]
    fn test_v3_add() {
        let a = V3::new(1.0, 2.0, 3.0);
        let b = V3::new(1.0, 2.0, 3.0);
        let c = V3::new(2.0, 4.0, 6.0);
        assert_eq!(a + b, c);
    }

    #[test]
    fn test_v3_add_assign() {
        let mut a = V3::new(1.0, 2.0, 3.0);
        let b = V3::new(1.0, 2.0, 3.0);
        let c = V3::new(2.0, 4.0, 6.0);
        a += b;
        assert_eq!(a, c);
    }

    #[test]
    fn test_v3_sub() {
        let a = V3::new(1.0, 2.0, 3.0);
        let b = V3::new(1.0, 2.0, 3.0);
        let c = V3::new(0.0, 0.0, 0.0);
        assert_eq!(a - b, c);
    }

    #[test]
    fn test_v3_sub_assign() {
        let mut a = V3::new(1.0, 2.0, 3.0);
        let b = V3::new(1.0, 2.0, 3.0);
        let c = V3::new(0.0, 0.0, 0.0);
        a -= b;
        assert_eq!(a, c);
    }

    #[test]
    fn test_v3_mul() {
        let a = V3::new(1.0, 2.0, 3.0);
        let b = 2.0;
        let c = V3::new(2.0, 4.0, 6.0);
        assert_eq!(a * b, c);
    }

    #[test]
    fn test_v3_mul_assign() {
        let mut a = V3::new(1.0, 2.0, 3.0);
        let b = 2.0;
        let c = V3::new(2.0, 4.0, 6.0);
        a *= b;
        assert_eq!(a, c);
    }

    #[test]
    fn test_v3_div() {
        let a = V3::new(2.0, 4.0, 6.0);
        let b = 2.0;
        let c = V3::new(1.0, 2.0, 3.0);
        assert_eq!(a / b, c);
    }

    #[test]
    fn test_v3_div_assign() {
        let mut a = V3::new(2.0, 4.0, 6.0);
        let b = 2.0;
        let c = V3::new(1.0, 2.0, 3.0);
        a /= b;
        assert_eq!(a, c);
    }

    #[test]
    fn test_v3_neg() {
        let a = V3::new(1.0, 2.0, 3.0);
        let b = V3::new(-1.0, -2.0, -3.0);
        assert_eq!(-a, b);
    }

    #[test]
    fn test_v3_dot() {
        let a = V3::new(1.0, 2.0, 3.0);
        let b = V3::new(1.0, 2.0, 3.0);
        let c = 14.0;
        assert_eq!(a * b, c);
    }

    #[test]
    fn test_v3_cross() {
        let a = V3::new(1.0, 0.0, 0.0);
        let b = V3::new(0.0, 1.0, 0.0);
        let c = V3::new(0.0, 0.0, 1.0);
        assert_eq!(a ^ b, c);
    }

    #[test]
    fn test_v3_cross_assign() {
        let mut a = V3::new(1.0, 0.0, 0.0);
        let b = V3::new(0.0, 1.0, 0.0);
        let c = V3::new(0.0, 0.0, 1.0);
        a ^= b;
        assert_eq!(a, c);
    }

    #[test]
    fn test_v3_length() {
        let a = V3::new(1.0, 2.0, 3.0);
        let b = 14.0_f32.sqrt();
        assert_eq!(a.length(), b);
    }

    #[test]
    fn test_v3_normalize() {
        let mut a = V3::new(1.0, 2.0, 3.0);
        let b = V3::new(
            1.0 / 14.0_f32.sqrt(),
            2.0 / 14.0_f32.sqrt(),
            3.0 / 14.0_f32.sqrt(),
        );
        assert_eq!(a.normalize(), b);
    }

    #[test]
    fn test_v3_get_normalized() {
        let a = V3::new(1.0, 2.0, 3.0);
        let b = V3::new(
            1.0 / 14.0_f32.sqrt(),
            2.0 / 14.0_f32.sqrt(),
            3.0 / 14.0_f32.sqrt(),
        );
        assert_eq!(a.get_normalized(), b);
    }

    #[test]
    fn test_lerp() {
        let a = V3::new(1.0, 2.0, 3.0);
        let b = V3::new(2.0, 4.0, 6.0);
        let c = V3::new(1.5, 3.0, 4.5);
        assert_eq!(lerp(a, b, 0.5), c);
    }

    #[test]
    fn test_m44_new() {
        let a = M44::new([
            [1.0, 2.0, 3.0, 4.0],
            [5.0, 6.0, 7.0, 8.0],
            [9.0, 10.0, 11.0, 12.0],
            [13.0, 14.0, 15.0, 16.0],
        ]);
        assert_eq!(a.m[0][0], 1.0);
        assert_eq!(a.m[0][1], 2.0);
        assert_eq!(a.m[0][2], 3.0);
        assert_eq!(a.m[0][3], 4.0);
        assert_eq!(a.m[1][0], 5.0);
        assert_eq!(a.m[1][1], 6.0);
        assert_eq!(a.m[1][2], 7.0);
        assert_eq!(a.m[1][3], 8.0);
        assert_eq!(a.m[2][0], 9.0);
        assert_eq!(a.m[2][1], 10.0);
        assert_eq!(a.m[2][2], 11.0);
        assert_eq!(a.m[2][3], 12.0);
        assert_eq!(a.m[3][0], 13.0);
        assert_eq!(a.m[3][1], 14.0);
        assert_eq!(a.m[3][2], 15.0);
        assert_eq!(a.m[3][3], 16.0);
    }

    #[test]
    fn test_m44_with_value() {
        let a = M44::with_value(1.0);
        let b = M44::new([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);
        assert_eq!(a, b);
    }

    #[test]
    fn test_m44_identity() {
        let a = M44::identity();
        let b = M44::new([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);
        assert_eq!(a, b);
    }

    #[test]
    fn test_m44_det() {
        let a = M44::new([
            [1.0, 2.0, 3.0, 4.0],
            [5.0, 6.0, 7.0, 8.0],
            [9.0, 10.0, 11.0, 12.0],
            [13.0, 14.0, 15.0, 16.0],
        ]);
        let b = 0.0;
        assert_eq!(a.det(), b);
    }

    #[test]
    fn test_m44_transpose() {
        let a = M44::new([
            [1.0, 2.0, 3.0, 4.0],
            [5.0, 6.0, 7.0, 8.0],
            [9.0, 10.0, 11.0, 12.0],
            [13.0, 14.0, 15.0, 16.0],
        ]);
        let b = M44::new([
            [1.0, 5.0, 9.0, 13.0],
            [2.0, 6.0, 10.0, 14.0],
            [3.0, 7.0, 11.0, 15.0],
            [4.0, 8.0, 12.0, 16.0],
        ]);
        assert_eq!(a.transpose(), b);
    }

    #[test]
    fn test_m44_mul() {
        let a = M44::new([
            [1.0, 2.0, 3.0, 4.0],
            [5.0, 6.0, 7.0, 8.0],
            [9.0, 10.0, 11.0, 12.0],
            [13.0, 14.0, 15.0, 16.0],
        ]);
        let b = M44::new([
            [1.0, 2.0, 3.0, 4.0],
            [5.0, 6.0, 7.0, 8.0],
            [9.0, 10.0, 11.0, 12.0],
            [13.0, 14.0, 15.0, 16.0],
        ]);
        let c = M44::new([
            [90.0, 100.0, 110.0, 120.0],
            [202.0, 228.0, 254.0, 280.0],
            [314.0, 356.0, 398.0, 440.0],
            [426.0, 484.0, 542.0, 600.0],
        ]);
        assert_eq!(a * b, c);
    }

    #[test]
    fn test_m44_mul_assign() {
        let mut a = M44::new([
            [1.0, 2.0, 3.0, 4.0],
            [5.0, 6.0, 7.0, 8.0],
            [9.0, 10.0, 11.0, 12.0],
            [13.0, 14.0, 15.0, 16.0],
        ]);
        let b = M44::new([
            [1.0, 2.0, 3.0, 4.0],
            [5.0, 6.0, 7.0, 8.0],
            [9.0, 10.0, 11.0, 12.0],
            [13.0, 14.0, 15.0, 16.0],
        ]);
        let c = M44::new([
            [90.0, 100.0, 110.0, 120.0],
            [202.0, 228.0, 254.0, 280.0],
            [314.0, 356.0, 398.0, 440.0],
            [426.0, 484.0, 542.0, 600.0],
        ]);
        a *= b;
        assert_eq!(a, c);
    }

    #[test]
    fn test_m44_translate() {
        let a = M44::new([
            [1.0, 2.0, 3.0, 4.0],
            [5.0, 6.0, 7.0, 8.0],
            [9.0, 10.0, 11.0, 12.0],
            [13.0, 14.0, 15.0, 16.0],
        ]);
        let b = V3::new(1.0, 2.0, 3.0);
        let c = M44::new([
            [1.0, 2.0, 3.0, 5.0],
            [5.0, 6.0, 7.0, 10.0],
            [9.0, 10.0, 11.0, 15.0],
            [13.0, 14.0, 15.0, 16.0],
        ]);
        assert_eq!(a.translate(b), c);
    }

    #[test]
    fn test_m44_scale() {
        let a = M44::new([
            [1.0, 2.0, 3.0, 4.0],
            [5.0, 6.0, 7.0, 8.0],
            [9.0, 10.0, 11.0, 12.0],
            [13.0, 14.0, 15.0, 16.0],
        ]);
        let b = V3::new(1.0, 2.0, 3.0);
        let c = M44::new([
            [1.0, 2.0, 3.0, 4.0],
            [5.0, 12.0, 7.0, 8.0],
            [9.0, 10.0, 33.0, 12.0],
            [13.0, 14.0, 15.0, 16.0],
        ]);
        assert_eq!(a.scale(b), c);
    }
}
