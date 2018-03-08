// Vector, 3-Floats, super-lightweight-robotics library
// 2003-2018 sergei lupashin
// License: BSD-3-clause

#pragma once

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <math.h>
#include "V3D.h"

#define NULL_VECTOR_F V3F(0.0,0.0,0.0)

inline void normalize_3(float* v){
  float mag = sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
  v[0] /= mag;
  v[1] /= mag;
  v[2] /= mag;
}

#pragma pack(push,1)

class V3F{
public:
  float x,y,z;
  V3F():x(0),y(0),z(0){};
	V3F(const float v):x(v),y(v),z(v){};
  V3F(const float X, const float Y, const float Z):x(X),y(Y),z(Z){};
  V3F(const float* d):x(d[0]),y(d[1]),z(d[2]){};
	explicit V3F(const double* d):x((float)d[0]),y((float)d[1]),z((float)d[2]){};
	explicit V3F(const V3D& d):x((float)d.x),y((float)d.y),z((float)d.z){};
  
  inline V3F operator+(const V3F b) const{    return V3F(x+b.x,y+b.y,z+b.z);  };
  inline V3F operator*(const V3F b) const{    return V3F(x*b.x,y*b.y,z*b.z);  };
  inline V3F operator/(const V3F b) const{    return V3F(x/b.x,y/b.y,z/b.z);  };
  inline V3F operator-(const V3F b) const{    return V3F(x-b.x,y-b.y,z-b.z);  };
  
  inline V3F operator+(const float b) const{    return V3F(x+b,y+b,z+b);  };
  inline V3F operator*(const float b) const{    return V3F(x*b,y*b,z*b);  };
  inline V3F operator/(const float b) const{    return V3F(x/b,y/b,z/b);  };
  inline V3F operator-(const float b) const{    return V3F(x-b,y-b,z-b);  };

  inline bool operator==(const V3F& b) const{ return (b.x==x && b.y==y && b.z==z);}
  inline bool operator!=(const V3F& b) const{ return (b.x!=x || b.y!=y || b.z!=z);}

  inline V3F operator/=(const V3F b){ *this = *this / b; return *this;}
  inline V3F operator*=(const V3F b){ *this = *this * b; return *this;}
  inline V3F operator+=(const V3F b){ *this = *this + b; return *this;}
  inline V3F operator-=(const V3F b){ *this = *this - b; return *this;}

  inline V3F sq() const{ return operator*(*this);}
	
	inline V3F operator - () const { return V3F(-x, -y, -z); }

  inline void constrain(float lowXYZ, float highXYZ)
  {
    constrain(lowXYZ, highXYZ, lowXYZ, highXYZ, lowXYZ, highXYZ);
  }
  inline void constrain(float lowX, float highX, float lowY, float highY, float lowZ, float highZ)
  {
    x = CONSTRAIN(x, lowX, highX);
    y = CONSTRAIN(y, lowY, highY);
    z = CONSTRAIN(z, lowZ, highZ);
  }

  inline float mag() const{return sqrtf(x*x+y*y+z*z);  }
	inline float magSq() const{return (x*x+y*y+z*z);  }
	inline float magXY() const{return sqrtf(x*x+y*y); }

	inline bool isZero() const{return (x==0.f && y==0.f && z==0.f);};

  inline void zero(){x=y=z=0.0;};

  inline float operator[](const int i) const {
    switch (i) {
      case 0: return x;
      case 1: return y;
      default: return z;
    }
  }
	
  inline float& operator[](const int i){
    switch(i){
      case 0: return x;
      case 1: return y;
      default: return z;
    }
  }

	inline const float* getArray() const { return &x; };
  inline void get(float* v) const {v[0] = x;    v[1] = y;    v[2] = z;};
	inline void get(float& X, float& Y, float& Z) const {X = x;    Y = y;    Z = z;};
  inline void set(float* v)       {x = v[0];    y = v[1];    z = v[2];};

  inline V3F cross(const V3F v) const{
	  V3F resVector;
	  resVector.x = y*v.z - z*v.y;
	  resVector.y = z*v.x - x*v.z;
	  resVector.z = x*v.y - y*v.x;
	  return resVector;
  }

  inline float dot(const V3F a) const{
    return x*a.x + y*a.y + z*a.z;
  }

  inline V3F norm() const{
    V3F res;
	  float l = mag();
	  if (l == 0.0f) return NULL_VECTOR_F;
    return operator/(l);
  }

	inline float sum() const{ return x+y+z;};

  inline float dist(const V3F b) const{    return operator-(b).mag();  }
	inline float dist_sq(const V3F b) const{    return operator-(b).sq().sum();  }

	inline float distXY(const V3F b) const{    return sqrtf((x-b.x)*(x-b.x)+(y-b.y)*(y-b.y));  }

	inline operator V3D() const
	{
		return V3D(x,y,z);
	}

};

inline V3F operator*(float a, V3F b)
{
  return V3F(a*b.x, a*b.y, a*b.z);
}

inline V3F operator/(float a, V3F b)
{
  return V3F(a / b.x, a / b.y, a / b.z);
}

inline V3F operator-(float a, V3F b)
{
  return V3F(a - b.x, a - b.y, a - b.z);
}

inline V3F operator+(float a, V3F b)
{
  return b + a;
}

#pragma pack(pop)

