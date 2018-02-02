// Vector, 3-Doubles, super-lightweight-robotics library
// 2003-2018 sergei lupashin
// License: BSD-3-clause

#pragma once

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <math.h>
#include "MathUtils.h"
#include <limits>
#include <string>
using namespace std;

#define NULL_VECTOR_D V3D(0.0,0.0,0.0)

inline void normalize_3(double* v){
  double mag = sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
  v[0] /= mag;
  v[1] /= mag;
  v[2] /= mag;
}

#pragma pack(push,1)

class V3D{
public:
  double x,y,z;
  V3D():x(0),y(0),z(0){};
	V3D(const int v):x(v),y(v),z(v){};
	V3D(const double v):x(v),y(v),z(v){};
  V3D(const double X, const double Y, const double Z):x(X),y(Y),z(Z){};
	V3D(const double* d):x(d[0]),y(d[1]),z(d[2]){};
  V3D(const float* d):x(d[0]),y(d[1]),z(d[2]){};
#ifdef BOOST_MSVC
	V3D(const boost::numeric::ublas::bounded_vector<double,3>& d):x(d(0)),y(d(1)),z(d(2)){};
	V3D(const boost::numeric::ublas::bounded_vector<float,3>& d):x(d(0)),y(d(1)),z(d(2)){};
#endif
  
  inline V3D operator+(const V3D b) const{    return V3D(x+b.x,y+b.y,z+b.z);  };
  inline V3D operator*(const V3D b) const{    return V3D(x*b.x,y*b.y,z*b.z);  };
  inline V3D operator/(const V3D b) const{    return V3D(x/b.x,y/b.y,z/b.z);  };
  inline V3D operator-(const V3D b) const{    return V3D(x-b.x,y-b.y,z-b.z);  };
  
  inline V3D operator+(const double b) const{    return V3D(x+b,y+b,z+b);  };
  inline V3D operator*(const double b) const{    return V3D(x*b,y*b,z*b);  };
  inline V3D operator/(const double b) const{    return V3D(x/b,y/b,z/b);  };
  inline V3D operator-(const double b) const{    return V3D(x-b,y-b,z-b);  };

  inline bool operator==(const V3D& b) const{ return (b.x==x && b.y==y && b.z==z);}
  inline bool operator!=(const V3D& b) const{ return (b.x!=x || b.y!=y || b.z!=z);}
	inline bool operator>(const V3D& b) const{ return (b.x>x || b.y>y || b.z>z);}
	inline bool operator<(const V3D& b) const{ return (b.x<x || b.y<y || b.z<z);}

  inline V3D operator/=(const V3D b){ *this = *this / b; return *this;}
  inline V3D operator*=(const V3D b){ *this = *this * b; return *this;}
  inline V3D operator+=(const V3D b){ *this = *this + b; return *this;}
  inline V3D operator-=(const V3D b){ *this = *this - b; return *this;}

  inline V3D sq() const{ return operator*(*this);}
  inline V3D sqrt() const{ return V3D(::sqrt(x),::sqrt(y),::sqrt(z));}
	
	inline V3D operator - () const { return V3D(-x, -y, -z); }

	inline void constrain(double lowXYZ,double highXYZ)
	{
		constrain(lowXYZ,highXYZ,lowXYZ,highXYZ,lowXYZ,highXYZ);
	}
	inline void constrain(double lowX,double highX, double lowY, double highY, double lowZ, double highZ)
	{
		x = CONSTRAIN(x,lowX,highX);
		y = CONSTRAIN(y,lowY,highY);
		z = CONSTRAIN(z,lowZ,highZ);
	}

  inline double mag() const{return ::sqrt(x*x+y*y+z*z);  }
	inline double magSq() const{return (x*x+y*y+z*z);  }
  inline double magXY() const{return ::sqrt(x*x+y*y); }
  inline double norminf() const{return MAX(MAX(fabs(x),fabs(y)),fabs(z));}

  inline bool isZero() const{return (x==0.0 && y==0.0 && z==0.0);};

  inline void zero(){x=y=z=0.0;};

  static V3D Inf() {
    return V3D(numeric_limits<double>::infinity(),numeric_limits<double>::infinity(),numeric_limits<double>::infinity());
  }

	// projects this vector onto b and return the magnitude of the resulting vector
	inline double projectMag(const V3D& b)
	{
		return (this->dot(b))/b.magSq();
	}

	// return projection of this vector onto b 
	inline V3D projectOnto(const V3D& b)
	{
		return b.norm() * (this->dot(b.norm()));
	}

  inline double operator[](const int i) const {
    switch (i) {
      case 0: return x;
      case 1: return y;
      default: return z;
    }
  }
	
  inline double& operator[](const int i){
    switch(i){
      case 0: return x;
      case 1: return y;
      default: return z;
    }
  }

	inline const double* getArray() const { return &x; };
  inline void get(double* v) const {v[0] = x;    v[1] = y;    v[2] = z;};
	inline void get(float* v) const {v[0] = (float)x;    v[1] = (float)y;    v[2] = (float)z;};
	inline void get(double& X, double& Y, double& Z) const {X = x;    Y = y;    Z = z;};
  inline void set(double* v)       {x = v[0];    y = v[1];    z = v[2];};

  inline V3D cross(const V3D v) const{
	  V3D resVector;
	  resVector.x = y*v.z - z*v.y;
	  resVector.y = z*v.x - x*v.z;
	  resVector.z = x*v.y - y*v.x;
	  return resVector;
  }

  inline double dot(const V3D a) const{
    return x*a.x + y*a.y + z*a.z;
  }

  inline V3D norm() const{
    V3D res;
	  double l = mag();
	  if (l == 0.0) return NULL_VECTOR_D;
    return operator/(l);
  }

	inline double sum() const{ return x+y+z;};

  inline double dist(const V3D b) const{    return operator-(b).mag();  }
	inline double dist_sq(const V3D b) const{    return operator-(b).sq().sum();  }

  inline double distXY(const V3D b) const{    return ::sqrt((x-b.x)*(x-b.x)+(y-b.y)*(y-b.y));  }

#ifdef _WIN32
  inline string ToString(const char* delim=" ") const
  {
    char buf[200];
    sprintf_s(buf,200,"%.3lf%s%.3lf%s%.3lf",x,delim,y,delim,z);
    return string(buf);
  }

  inline string ToString_FullPrecision() const
  {
    char buf[200];
    sprintf_s(buf,200,"%lf %lf %lf",x,y,z);
    return string(buf);
  }
#endif
};

inline V3D operator*(double a, V3D b)
{
	return V3D(a*b.x,a*b.y,a*b.z);
}

inline V3D operator/(double a, V3D b)
{
	return V3D(a/b.x,a/b.y,a/b.z);
}

inline V3D operator-(double a, V3D b)
{
	return V3D(a-b.x,a-b.y,a-b.z);
}

inline V3D operator+(double a, V3D b)
{
	return b+a;
}

#pragma pack(pop)

