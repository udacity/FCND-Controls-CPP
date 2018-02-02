// Vector, 4-Doubles, super-lightweight-robotics library
// 2003-2018 sergei lupashin
// License: BSD-3-clause

#pragma once

struct V4D
{
  V4D() { v[0] = v[1] = v[2] = v[3] = 0; }

  V4D(double a, double b, double c, double d)
  {
    v[0] = a;
    v[1] = b;
    v[2] = c;
    v[3] = d;
  }

  V4D(const V4D& b)
  {
    v[0] = b.v[0];
    v[1] = b.v[1];
    v[2] = b.v[2];
    v[3] = b.v[3];
  }

  V4D(const double b[4])
  {
    v[0] = b[0];
    v[1] = b[1];
    v[2] = b[2];
    v[3] = b[3];
  }

  V4D(const float b[4])
  {
    v[0] = b[0];
    v[1] = b[1];
    v[2] = b[2];
    v[3] = b[3];
  }

  V4D operator*(double d) const
  {
    return V4D(v[0] * d, v[1] * d, v[2] * d, v[3] * d);
  }

  V4D operator/(double d) const
  {
    return V4D(v[0] / d, v[1] / d, v[2] / d, v[3] / d);
  }

  V4D operator+(V4D d) const
  {
    return V4D(v[0] + d[0], v[1] + d[1], v[2] + d[2], v[3] + d[3]);
  }

  V4D operator-(V4D d) const
  {
    return V4D(v[0] - d[0], v[1] - d[1], v[2] - d[2], v[3] - d[3]);
  }

  double operator[](int i) const { return v[i]; }
  double operator()(int i) const { return v[i]; }
  double & operator[](int i) { return v[i]; }
  double & operator()(int i) { return v[i]; }

  double v[4];
};

inline V4D element_prod(const V4D& a, const V4D& b)
{
  return V4D(a.v[0] * b.v[0], a.v[1] * b.v[1], a.v[2] * b.v[2], a.v[3] * b.v[3]);
}

inline double sum(const V4D& a)
{
  return a.v[0] + a.v[1] + a.v[2] + a.v[3];
}

inline double norm_2(const V4D& a)
{
  return a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3];

}