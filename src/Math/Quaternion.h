// Quaternion Class, super-lightweight-robotics library
// License: BSD-3-clause

#pragma once 

#include "Constants.h"
#include "V3D.h"
#include "V3F.h"
#include "Mat3x3F.h"
#include "V4D.h"

namespace SLR{

template<typename T=double>
class Quaternion
{
  // NOTES ON INTERNAL REPRESENTATION:
  // A quaternion is 4 numbers: [w,x,y,z]. For representing rotations, we assume 
  // that the second norm is 1: sqrt(w*w+x*x+y*y+z*z)=1. w is the real component 
  // and x,y,z are the imaginary components. Really, w = cos(theta/2) where theta
  // is the angle by which to rotate about the axis [x,y,z]/sin(theta) to achieve 
  // the rotation described by the quaternion. See wikipedia, etc for more info.
  
  // _q contains the numbers [w,x,y,z], in that order.

  // A quaternion of rotation represents the rotation by angle THETA around axis
  // N from the Initial reference frame to the Final reference frame. We define
  // the quaternion multiplication so that it works like successive applications
  // of rotation matrices. 
  // Thus, if Q10 is the representation of frame 1 w.r.t. frame 0, and likewise
  // Q21 is the representation of frame 2 w.r.t. frame 1, we have Q20 = Q21*Q10,
  // i.e. the inner indices annihilate one another. This is equivalent to rotation 
  // matrix multiplication: Let T10, T21 and T20 represent the rotation matrices 
  // of frame 1 w.r.t. 0, and so forth, then T20 = T21*T10. 

  // To rotate a vector into the frame of the quaternion, use the RotationMatrix_BwrtI()
  // function. I.e. for Q10 as above, Q10.RotationMatrix_BwrtI()*somevec will 
  // give the representation of somevec in frame 1. 
  
public:
  Quaternion()
  {
    _q[0] = 1;
    _q[1] = 0;
    _q[2] = 0;
    _q[3] = 0;
  };

  Quaternion(T w, T x, T y, T z)
  {
    _q[0]=w;
    _q[1]=x;
    _q[2]=y;
    _q[3]=z;
  }

  static Quaternion Identity(void){return Quaternion(1,0,0,0);};

  // for unit quaternions adjoint == reciprocal == conjugate == inverse.
  // for the curious, the general definition for q^(-1)=adjoint(q)/(|q|^2) where
  // adjoint([q0 q1 q2 q3]) is [q0 -q1 -q2 -q3]
  Quaternion Conjugate() const
  {
    return Quaternion(_q[0],-_q[1],-_q[2],-_q[3]);
  }
  
  // ASSUME: Unit quaternion
  Quaternion Inverse() const
  {
    return Quaternion(_q[0],-_q[1],-_q[2],-_q[3]);
  }

  // quaternion multiplication: q*p <- this corresponds to a rotation p followed by rotation q?
  Quaternion operator*(const Quaternion& p) const
  {
    const T* q = _q; // for convenience
    T c0 = q[0]*p[0] - q[1]*p[1] - q[2]*p[2] - q[3]*p[3]; // 0 -1 -2 -3
    T c1 = q[1]*p[0] + q[0]*p[1] + q[3]*p[2] - q[2]*p[3]; // 1  0  3 -2
    T c2 = q[2]*p[0] - q[3]*p[1] + q[0]*p[2] + q[1]*p[3]; // 2 -3  0  1
    T c3 = q[3]*p[0] + q[2]*p[1] - q[1]*p[2] + q[0]*p[3]; // 3  2 -1  0
    return Quaternion(c0,c1,c2,c3);
  }

	// CHECKED
  static Quaternion FromEuler123_RPY(T roll, T pitch, T yaw)
  {
    // the roll,pitch,yaw angles are the "usual SLR sequence" Euler angles:
		// 1. rotate inertial frame about global-Z for yaw
		// 2. rotate resulting frame for pitch
		// 3. rotate resulting frame for roll

    // Diebels formula 84, page 12. 
    roll /= 2.f;
    pitch /= 2.f;
    yaw /= 2.f;
  
    T q0 = cos(roll) * cos(pitch) * cos(yaw)   + sin(roll)  * sin(pitch) * sin(yaw);
    T q1 =-cos(roll) * sin(pitch) * sin(yaw)   + cos(pitch) * cos(yaw)   * sin(roll);
    T q2 = cos(roll) * cos(yaw)   * sin(pitch) + sin(roll)  * cos(pitch) * sin(yaw);
    T q3 = cos(roll) * cos(pitch) * sin(yaw)   - sin(roll)  * cos(yaw)   * sin(pitch);

    // Invert the quaternion to correspond with our usual definition!
		// The returned quaternion now represents the attitude of a vehicle with the above angles
    return Quaternion(q0,q1,q2,q3);
  }
  
	// CHECKED
  static Quaternion FromAxisAngle(const V3D axisAngle)
  {
    const T theta = (T)axisAngle.mag();
    if(!theta) return Identity();

    Quaternion qout;
    qout[0] = cos(theta/2.f);
    qout[1] = sin(theta/2.f)*(T)axisAngle.x/theta;
    qout[2] = sin(theta/2.f)*(T)axisAngle.y/theta;
    qout[3] = sin(theta/2.f)*(T)axisAngle.z/theta;
    return qout;
  };

  void GetAxisAndAngle(V3F& axis, float &angle)
  {
    angle = acos(_q[0])*2.f;
    float tmp = sinf(angle / 2.f);
    axis = V3F(_q[1] / tmp, _q[2] / tmp, _q[3] / tmp);
  }

  static Quaternion FromGibbsVector(const V3D gv)
  {
    Quaternion qout;
    qout[0] =  1.f/sqrt(1+(T)gv.magSq());
    qout[1] = gv.x/sqrt(1+(T)gv.magSq());
    qout[2] = gv.y/sqrt(1+(T)gv.magSq());
    qout[3] = gv.z/sqrt(1+(T)gv.magSq());
    return qout;
  }

	// CHECKED
	static Quaternion FromAxisAngle_nocheck(const V3F axis, float angle)
	{
		// assume axis is normalized
		angle /= 2.f;
		const T cosA = cos(angle);
		const T sinA = sin(angle);
		return Quaternion(cosA,sinA*axis.x,sinA*axis.y,sinA*axis.z);
	}

	// CHECKED
  static Quaternion FromEulerYPR(const T& y, const T& p, const T& r)
  {//NB! rotation: 3-2-1 yaw,pitch,roll
    Quaternion q;
    q[0] = cos(0.5f*y)*cos(0.5f*p)*cos(0.5f*r) + sin(0.5f*y)*sin(0.5f*p)*sin(0.5f*r);
    q[1] = cos(0.5f*y)*cos(0.5f*p)*sin(0.5f*r) - sin(0.5f*y)*sin(0.5f*p)*cos(0.5f*r);
    q[2] = cos(0.5f*y)*sin(0.5f*p)*cos(0.5f*r) + sin(0.5f*y)*cos(0.5f*p)*sin(0.5f*r);
    q[3] = sin(0.5f*y)*cos(0.5f*p)*cos(0.5f*r) - cos(0.5f*y)*sin(0.5f*p)*sin(0.5f*r);
    return q;
  };

	T& operator[](uint8_t i){return _q[i];}
  const T& operator[](uint8_t i) const{return _q[i];}

  static Quaternion FromRotmatrix(Mat3x3F Rotmat) 
  {	
	  Quaternion q;
	  q[0] = 0.5f*sqrt(1 + (T)Rotmat[0] + (T)Rotmat[4] + (T)Rotmat[8]);
	  q[1] = -1/(4*q[0])*((T)Rotmat[7] - (T)Rotmat[5]);
	  q[2] = -1/(4*q[0])*((T)Rotmat[2] - (T)Rotmat[6]);
	  q[3] = -1/(4*q[0])*((T)Rotmat[3] - (T)Rotmat[1]);
	  return q;
  };  

  Mat3x3F RotationMatrix_IwrtB(void) const
  {
		//transformation matrix of inertial w.r.t. the body -- v_I = RotationMatrix_IwrtB*v_B
    V3F qvec = V3F(_q[1],_q[2],_q[3]);
    Mat3x3F Sk_q =  Mat3x3F::SkewSymmetric(qvec);// Build the skew symmetric matrix of the imaginary part of quaternion
    return Mat3x3F::Ident()*(_q[0]*_q[0] - qvec.magSq()) + Mat3x3F::OuterProduct(qvec,qvec)*2  + Sk_q*2*_q[0];
  };

  Mat3x3F RotationMatrix_BwrtI(void) const
  {
		//transformation matrix of body w.r.t. inertial		
    Mat3x3F temp = RotationMatrix_IwrtB();
    temp.Transpose();
    return temp;
  };

	// R is assumed to be row-major
	// v_I = R * v_B
	void RotationMatrix_IwrtB(float* R) const
	{
		const float& q0=-_q[0]; const float r0=q0*q0;
		const float& q1=_q[1]; const float r1=q1*q1;
		const float& q2=_q[2]; const float r2=q2*q2;
		const float& q3=_q[3]; const float r3=q3*q3;

		R[0] = r0 + r1 - r2 - r3;
		R[1] = 2*q1*q2 + 2*q0*q3;
		R[2] = 2*q1*q3 - 2*q0*q2;

		R[3] = 2*q1*q2 - 2*q0*q3;
		R[4] = r0 - r1 + r2 - r3;
		R[5] = 2*q2*q3 + 2*q0*q1;

		R[6] = 2*q1*q3 + 2*q0*q2;
		R[7] = 2*q2*q3 - 2*q0*q1;
		R[8] = r0 - r1 - r2 + r3;
	}

	// R is assumed to be row-major
	// v_B = R * v_I
	void RotationMatrix_BwrtI(float* R) const
	{
		const float& q0=_q[0]; const float r0=q0*q0;
		const float& q1=_q[1]; const float r1=q1*q1;
		const float& q2=_q[2]; const float r2=q2*q2;
		const float& q3=_q[3]; const float r3=q3*q3;

		R[0] = r0 + r1 - r2 - r3;
		R[1] = 2*q1*q2 + 2*q0*q3;
		R[2] = 2*q1*q3 - 2*q0*q2;

		R[3] = 2*q1*q2 - 2*q0*q3;
		R[4] = r0 - r1 + r2 - r3;
		R[5] = 2*q2*q3 + 2*q0*q1;

		R[6] = 2*q1*q3 + 2*q0*q2;
		R[7] = 2*q2*q3 - 2*q0*q1;
		R[8] = r0 - r1 - r2 + r3;
	}

	V3F RotationMatrix_IwrtB_singleColumn(uint8_t col) const
	{
		V3F R;
		const float& q0=-_q[0]; const float r0=q0*q0;
		const float& q1=_q[1]; const float r1=q1*q1;
		const float& q2=_q[2]; const float r2=q2*q2;
		const float& q3=_q[3]; const float r3=q3*q3;

		if(col==0)
		{
			R[0] = r0 + r1 - r2 - r3;
			R[1] = 2*q1*q2 - 2*q0*q3;
			R[2] = 2*q1*q3 + 2*q0*q2;
		}
		else if(col==1)
		{
			R[0] = 2*q1*q2 + 2*q0*q3;			
			R[1] = r0 - r1 + r2 - r3;
			R[2] = 2*q2*q3 - 2*q0*q1;			
		}
		else if(col==2)
		{
			R[0] = 2*q1*q3 - 2*q0*q2;
			R[1] = 2*q2*q3 + 2*q0*q1;
			R[2] = r0 - r1 - r2 + r3;
		}
		return R;
	}
	
	// rotates a vector in body coordinates to global coords
	V3F Rotate_BtoI(const V3F& in) const
	{
		float R[9];
		RotationMatrix_IwrtB(R);

		V3F ret(R[0]*in[0] + R[1]*in[1] + R[2]*in[2],
			      R[3]*in[0] + R[4]*in[1] + R[5]*in[2],
						R[6]*in[0] + R[7]*in[1] + R[8]*in[2]);

		return ret;
	}

	// rotates a vector in inertial coords to body coords
	V3F Rotate_ItoB(const V3F& in) const
	{
		float R[9];
		RotationMatrix_BwrtI(R);

		V3F ret(R[0]*in[0] + R[1]*in[1] + R[2]*in[2],
			      R[3]*in[0] + R[4]*in[1] + R[5]*in[2],
						R[6]*in[0] + R[7]*in[1] + R[8]*in[2]);

		return ret;
	}

	inline float RotationMatrix_IwrtB_SingleElement(const uint8_t row, const uint8_t col) const
	{
		const float& q0=-_q[0]; 
		const float& q1=_q[1]; 
		const float& q2=_q[2]; 
		const float& q3=_q[3];
		switch(col)
		{
		case 0:
			switch(row)
			{
				case 0:  return q0*q0 + q1*q1 - q2*q2 - q3*q3;
				case 1:  return 2*q1*q2 - 2*q0*q3;
				default: return 2*q1*q3 + 2*q0*q2;
			};
		case 1:
			switch(row)
			{
				case 0:	return 2*q1*q2 + 2*q0*q3;			
				case 1: return q0*q0 - q1*q1 + q2*q2 - q3*q3;
				default:return 2*q2*q3 - 2*q0*q1;		
			};
		default:
			switch(row)
			{
				case 0:	return  2*q1*q3 - 2*q0*q2;
				case 1: return  2*q2*q3 + 2*q0*q1;
				default:return q0*q0 - q1*q1 - q2*q2 + q3*q3;
			};
		};
	}

  V3D ToGibbsVector(void) const 
  {
    if(!_q[0]) return V3D(_q[1],_q[2],_q[3])*1e20;//otherwise have infinite Gibb's vector

    return V3D(_q[1],_q[2],_q[3])/_q[0];
  };

	// CHECKED
  V3D ToEulerYPR(void) const
  {
    T y,p,r;
    
    const T div1 = _q[0]*_q[0] + _q[1]*_q[1] - _q[2]*_q[2] - _q[3]*_q[3];
    y = atan2(2*(_q[1]*_q[2]+_q[0]*_q[3]),div1);

    p = asin(-2*(_q[1]*_q[3] - _q[0]*_q[2]));

    const T div2 = _q[0]*_q[0] - _q[1]*_q[1] - _q[2]*_q[2] + _q[3]*_q[3];
    r = atan2(2*(_q[2]*_q[3]+_q[0]*_q[1]),div2);

    return V3D(y,p,r);
  }

  // convenience functions for Udacity
  float Yaw() const
  {
    const T div1 = _q[0] * _q[0] + _q[1] * _q[1] - _q[2] * _q[2] - _q[3] * _q[3];
    return atan2(2 * (_q[1] * _q[2] + _q[0] * _q[3]), div1);
  }

  float Pitch() const
  {
    return asin(-2 * (_q[1] * _q[3] - _q[0] * _q[2]));
  }

  float Roll() const
  {
    const T div2 = _q[0] * _q[0] - _q[1] * _q[1] - _q[2] * _q[2] + _q[3] * _q[3];
    return atan2(2 * (_q[2] * _q[3] + _q[0] * _q[1]), div2);
  }


	V3D ToEulerRPY(void) const
  {
		V3D ret = ToEulerYPR();
		return V3D(ret.z,ret.y,ret.x);
  }
	
	// CHECKED
	void ToEulerYPR(T& r, T& p, T& y) const
  {    
    const T div1 = _q[0]*_q[0] + _q[1]*_q[1] - _q[2]*_q[2] - _q[3]*_q[3];
    if(div1) y = atan2(2.f*(_q[1]*_q[2]+_q[0]*_q[3]),div1);
    else y = F_PI;//singular, +/- pi is equivalent

    p = asin(-2*(_q[1]*_q[3] - _q[0]*_q[2]));

    const T div2 = _q[0]*_q[0] - _q[1]*_q[1] - _q[2]*_q[2] + _q[3]*_q[3];
    if(div2) r = atan2(2.f*(_q[2]*_q[3]+_q[0]*_q[1]),div2);
    else r = F_PI;//singular, +/- pi is equivalent
  }


	// CHECKED
  inline Quaternion IntegrateBodyRate(const V3D pqr, const double dt) //body rates must be expressed in the body coordinate frame!
  {
    *this = Quaternion::FromAxisAngle(pqr*dt)*(*this);
    return *this;
  };

	// CHECKED
	inline Quaternion IntegrateBodyRate_fast(const V3F pqr, const float half_dt) //body rates must be expressed in the body coordinate frame!
  {
		// half-pqr-dt values
    const float p = -half_dt*pqr[0];
		const float q = -half_dt*pqr[1];
		const float r = -half_dt*pqr[2];

		const float qdot0 =         -p*_q[1] -q*_q[2] -r*_q[3];
		const float qdot1 = p*_q[0]          +r*_q[2] -q*_q[3];
		const float qdot2 = q*_q[0] -r*_q[1]          +p*_q[3];
		const float qdot3 = r*_q[0] +q*_q[1] -p*_q[2]         ;

		return Quaternion(_q[0]-qdot0,_q[1]-qdot1,_q[2]-qdot2,_q[3]-qdot3).Normalise();
  }

	// CHECKED
	inline Quaternion IntegrateBodyRate_fast(float p, float q, float r, const float half_dt) //body rates must be expressed in the body coordinate frame!
  {
		// half-pqr-dt values
    p *= -half_dt;
    q *= -half_dt;
    r *= -half_dt;

		const float qdot0 =         -p*_q[1] -q*_q[2] -r*_q[3];
		const float qdot1 = p*_q[0]          +r*_q[2] -q*_q[3];
		const float qdot2 = q*_q[0] -r*_q[1]          +p*_q[3];
		const float qdot3 = r*_q[0] +q*_q[1] -p*_q[2]         ;

		return Quaternion(_q[0]-qdot0,_q[1]-qdot1,_q[2]-qdot2,_q[3]-qdot3).Normalise();
  }

  inline T NormSq(void) const{ return _q[0]*_q[0]+_q[1]*_q[1]+_q[2]*_q[2]+_q[3]*_q[3];};
  inline T Norm(void) const{ return sqrt(NormSq());};

  inline Quaternion Normalise(void)
  { 
		const T n = Norm();
    if(n==0)//doesn't really make sense
    {
      *this = Identity();
    }
    else 
    {
      for(uint8_t i=0;i<4;i++) _q[i] = _q[i]/n;
    }
    return *this;
  }

	// CHECKED
	Quaternion AlignSigns(const Quaternion& b) const
	{
		const T dot = _q[1]*b._q[1]+_q[2]*b._q[2]+_q[3]*b._q[3];
		if(dot<0)
		{
			return Quaternion(-_q[0],-_q[1],-_q[2],-_q[3]);
		}
		return *this;
	}


  // interpolate between two quaternions along the torque-minimal path between q0 and q1
  // see http://number-none.com/product/Understanding%20Slerp,%20Then%20Not%20Using%20It/
  // this is probably not the most efficient way to implement it
  // q0 (self) and q1 should be unit quaternions.
  // t should be between 0 and 1 (it's constrained internally)
  // t = 0 returns q0; t=1 returns q1; something inbetween returns something inbetween
  Quaternion Interpolate_SLERP(const Quaternion& q1, double t)
  {
    // Compute the cosine of the angle between the two vectors.
    Quaternion q0 = *this;
    V4D b0(_q);
    V4D b1(q1[0], q1[1], q1[2], q1[3]);

    double dot = _q[0]*q1[0] + _q[1]*q1[1] + _q[2]*q1[2] + _q[3]*q1[3];

    t = CONSTRAIN(t, 0, 1);

    const double DOT_THRESHOLD = 0.9995;
    if (dot > DOT_THRESHOLD) {
      // If the inputs are too close for comfort, linearly interpolate
      // and normalize the result.
      
      V4D res = b0 + (b1 - b0)*t;
      res = res / norm_2(res);
      return Quaternion((float)res[0], (float)res[1], (float)res[2], (float)res[3]);

      //Quaternion result = q0 + (q1 + -q0)*t;
      //result /= norm_2(result);
      //return result;
    }

    dot = CONSTRAIN(dot, -1, 1);	// Robustness: Stay within domain of acos()
    double theta_0 = acos(dot);  // theta_0 = angle between input vectors
    double theta = theta_0*t;    // theta = angle between v0 and result 

    //Quaternion q2 = q1 + -q0*dot;
    //q2 /= norm_2(q2);              // { q0, q2 } is now an orthonormal basis
    //return q0*cos(theta) + q2*sin(theta);

    V4D b2 = b1 - b1*dot;
    b2 = b2 / norm_2(b2);
    V4D res = b0*cos(theta) + b2 * sin(theta);
    return Quaternion((float)res[0], (float)res[1], (float)res[2], (float)res[3]);
  }

#ifdef _WIN32
  string ToString() const
  {
    char buf[100];
    sprintf_s(buf,100,"%.3lf %.3lf %.3lf %.3lf",_q[0],_q[1],_q[2],_q[3]);
    return string(buf);
  }

  string ToStringFP() const //full precision
  {
    char buf[200];
    if(_q[0] >= 0) sprintf_s(buf,200,"%.3lf,%.3lf,%.3lf,%.3lf",_q[0],_q[1],_q[2],_q[3]);
    else sprintf_s(buf,200,"%.3lf,%.3lf,%.3lf,%.3lf",-_q[0],-_q[1],-_q[2],-_q[3]);
    return string(buf);
  }
#endif

protected:
  T _q[4];
};
} // namespace SLR
