// 3x3 Matrix of Doubles, super-lightweight-robotics library
// License: BSD-3-clause

#pragma once

#include <algorithm> // swap
#include <assert.h>
#include "V3D.h"

// class for operating with 3-by-3 matricies of doubles
// compatible with V3D!
// assumes row-major ordering:
// 0  1  2
// 3  4  5
// 6  7  8

class Mat3x3D
{
public:
	Mat3x3D()
	{
		for(unsigned char i=0;i<9;i++) _v[i]=0;		
		_v[0] = _v[4] = _v[8] = 1.0; // diagonal!
	}

	Mat3x3D(const double v[9])
	{
		for(unsigned char i=0;i<9;i++) _v[i]=v[i];
	}

	static Mat3x3D Zeros()
	{
		double v[9] = {0,0,0,0,0,0,0,0,0};
		return Mat3x3D(v);
	}

	static Mat3x3D Ident()
	{
		double v[9] = {1,0,0,0,1,0,0,0,1};
		return Mat3x3D(v);
	}

	void Transpose()
	{
		std::swap(_v[1],_v[3]);
		std::swap(_v[2],_v[6]);
		std::swap(_v[5],_v[7]);
	}

	Mat3x3D RetTranspose() //Returns transpose but leaves original matrix unchanged
	{
		Mat3x3D res = *this;
		std::swap(res._v[1],res._v[3]);
		std::swap(res._v[2],res._v[6]);
		std::swap(res._v[5],res._v[7]);
		
		return res;
	}

	double Trace() {
		double res = _v[0] + _v[4] + _v[8];
		return res;
	}

	Mat3x3D operator*(const Mat3x3D& b)
	{
		double res[9] = {0,0,0,0,0,0,0,0,0};
		for(unsigned char r=0;r<3;r++)
		{
			for(unsigned char c=0;c<3;c++)
			{
				res[r*3+c] += _v[r*3]*b._v[0*3+c] + _v[r*3+1]*b._v[1*3+c] + _v[r*3+2]*b._v[2*3+c];
			}
		}
		return Mat3x3D(res);
	}

	Mat3x3D operator*(const double& b)
	{
		double res[9] = {0,0,0,0,0,0,0,0,0};
		for(unsigned char i=0;i<9;i++) res[i] = b*_v[i];

		return Mat3x3D(res);
	}

	static Mat3x3D Rotation(V3D vec, double theta)
	{
		vec = vec.norm();
		double c = cos(theta);
		double s = sin(theta);
		double t = (1.0-c);
		double X = vec.x; double X2 = X*X;
		double Y = vec.y; double Y2 = Y*Y;
		double Z = vec.z; double Z2 = Z*Z;
		
		double v[9];
		v[0] = t*X2 + c;
		v[1] = t*X*Y + s*Z;
		v[2] = t*X*Z - s*Y;
		v[3] = t*X*Y - s*Z;
		v[4] = t*Y2 + c;
		v[5] = t*Y*Z + s*X;
		v[6] = t*X*Z + s*Y; //Corrected from v[6] = t*X*Y + s*Y; Luzius; 7.10.2010
		v[7] = t*Y*Z - s*X;
		v[8] = t*Z2 + c;

		return Mat3x3D(v);
	}

	static Mat3x3D SkewSymmetric(const double a, const double b, const double c)
	{
		double v[9];
		v[0] = 0.0;
		v[1] = -c;
		v[2] = b;

		v[3] = c;
		v[4] = 0.0;
		v[5] = -a;
		
		v[6] = -b;
		v[7] = a;
		v[8] = 0.0;
		
		return Mat3x3D(v);
	}

	static Mat3x3D SkewSymmetric(const V3D& w){return Mat3x3D::SkewSymmetric(w.x,w.y,w.z);}

	static Mat3x3D OuterProduct(const V3D a, const V3D b)//a^T*b
	{
		double v[9];
		v[0] = a.x*b.x;
		v[1] = a.x*b.y;
		v[2] = a.x*b.z;

		v[3] = a.y*b.x;
		v[4] = a.y*b.y;
		v[5] = a.y*b.z;

 		v[6] = a.z*b.x;
		v[7] = a.z*b.y;
		v[8] = a.z*b.z;

		return Mat3x3D(v);
	}

	V3D operator*(const V3D& a) const
	{
		V3D ret;
		ret[0] = a[0]*_v[0] + a[1]*_v[1] + a[2]*_v[2];
		ret[1] = a[0]*_v[3] + a[1]*_v[4] + a[2]*_v[5];
		ret[2] = a[0]*_v[6] + a[1]*_v[7] + a[2]*_v[8];
		return ret;
	}

	Mat3x3D operator+(const Mat3x3D& a)
	{
		Mat3x3D ret;
		for(unsigned char i=0;i<9;i++)
		{
			ret._v[i] = _v[i]+a._v[i];
		}
		return ret;
	}


	Mat3x3D operator-(const Mat3x3D& a)
	{
		Mat3x3D ret;
		for(unsigned char i=0;i<9;i++)
		{
			ret._v[i] = _v[i]-a._v[i];
		}
		return ret;
	}

	Mat3x3D operator/(const double a)
	{
		Mat3x3D ret;
		for(unsigned char i=0;i<9;i++)
		{
			ret._v[i] = _v[i]/a;
		}
		return ret;
	}

	double& operator()(unsigned char r, unsigned char c)
	{
		assert(r<3 && c<3);
		return _v[r*3+c];
	}

	double& operator[](int i)
	{
		assert(i>=0 && i<9);
		return _v[i];
	}

	const double& operator[](int i) const
	{
		assert(i>=0 && i<9);
		return _v[i];
	}

	 	//max element of 3x3 matrix, by Christoph Kammer
    double MaxEl(void) {
  		double temp=abs(_v[0]);
    	int num = 0;
	  	for(int i=1;i<9;i++) {
	  		if(temp<abs(_v[i])){
	  			temp = abs(_v[i]);
	 			num = i;
	  		}	
	    }
	    return _v[num];
    }	
 
 
	//max element of 3x3 matrix (absolute value), by Christoph Kammer
  	double AbsMaxEl(void) {
	  	double temp=abs(_v[0]);
	 	for(int i=1;i<9;i++) {
	  		if(temp<abs(_v[i])){
	  			temp = abs(_v[i]);
	  		}	
	    }
	    return temp;
	}	
	
	void SetValues(double v[9]) {
		for(unsigned char i=0;i<9;i++) _v[i]=v[i];
	}
	void SetZero() {
		for(unsigned char i=0;i<9;i++) _v[i]=0;
	}

	Mat3x3D Pointdot(const Mat3x3D& b) { //multiply matrices elementwise (Matlab: A.*B)
		double res[9]= {0,0,0,0,0,0,0,0,0};
		for(int i=0;i<9;i++) {
			res[i] = _v[i]*b._v[i];
		}
		return Mat3x3D(res);
	}

	double SumEl() {
		double res=_v[0];
		for(int i=1;i<9;i++){
			res += _v[i];
		}
		return res;
	}

	//cross product of vector with each column of a matrix, returns a matrix (vec x Mat)
	// side = 0 <-> vec x Mat
	// side = 1 <-> Mat x vec
	Mat3x3D Cross(const V3D vec, int side) {
		V3D tmp1(_v[0],_v[3],_v[6]);
		V3D tmp2(_v[1],_v[4],_v[7]);
		V3D tmp3(_v[2],_v[5],_v[8]);
		double v[9];
		if(side==0) {
			tmp1 = vec.cross(tmp1);
			tmp2 = vec.cross(tmp2);
			tmp3 = vec.cross(tmp3);
		}
		else if(side==1) {
			tmp1 = tmp1.cross(vec);
			tmp2 = tmp2.cross(vec);
			tmp3 = tmp3.cross(vec);
		}
		tmp1.get(v[0],v[3],v[6]);
		tmp2.get(v[1],v[4],v[7]);
		tmp3.get(v[2],v[5],v[8]);
		return Mat3x3D(v);
	}

	Mat3x3D ExtProduct(const Mat3x3D& b) { //expression for cross product of 2 matrices (you most likely will never need this ;P)
		double res[9] = {0,0,0,0,0,0,0,0,0};
		res[0] = _v[4]*b._v[8] + _v[8]*b._v[4] - _v[5]*b._v[7] - _v[7]*b._v[5];
		res[4] = _v[0]*b._v[8] + _v[8]*b._v[0] - _v[2]*b._v[6] - _v[6]*b._v[2];
		res[8] = _v[0]*b._v[4] + _v[4]*b._v[0] - _v[1]*b._v[3] - _v[3]*b._v[1];

		res[1] = _v[5]*b._v[6] + _v[6]*b._v[5] - _v[8]*b._v[3] - _v[3]*b._v[8];
		res[2] = _v[7]*b._v[3] + _v[3]*b._v[7] - _v[4]*b._v[6] - _v[6]*b._v[4];
		res[3] = _v[2]*b._v[7] + _v[7]*b._v[2] - _v[8]*b._v[1] - _v[1]*b._v[8];
		res[5] = _v[6]*b._v[1] + _v[1]*b._v[6] - _v[0]*b._v[7] - _v[7]*b._v[0];
		res[6] = _v[1]*b._v[5] + _v[5]*b._v[1] - _v[4]*b._v[2] - _v[2]*b._v[4];
		res[7] = _v[3]*b._v[2] + _v[2]*b._v[3] - _v[0]*b._v[5] - _v[5]*b._v[0];
		
		return Mat3x3D(res);
	}

	double Determinant() {
		double res = 0;
		res += _v[0]*(_v[4]*_v[8]-_v[7]*_v[5])
			 - _v[3]*(_v[1]*_v[8]-_v[7]*_v[2])
			 + _v[6]*(_v[1]*_v[5]-_v[4]*_v[2]);
		
		return res;
	}

	Mat3x3D Inverse(void) {
		Mat3x3D res;
		double  det = this->Determinant();
		res[0] = (_v[4]*_v[8]-_v[7]*_v[5])/det;
		res[1] = -(_v[1]*_v[8]-_v[7]*_v[2])/det;
		res[2] = (_v[1]*_v[5]-_v[4]*_v[2])/det;

		res[3] = -(_v[3]*_v[8]-_v[6]*_v[5])/det;
		res[4] = (_v[0]*_v[8]-_v[6]*_v[2])/det;
		res[5] = -(_v[0]*_v[5]-_v[3]*_v[2])/det;

		res[6] = (_v[3]*_v[7]-_v[6]*_v[4])/det;
		res[7] = -(_v[0]*_v[7]-_v[6]*_v[1])/det;
		res[8] = (_v[0]*_v[4]-_v[3]*_v[1])/det;

		return res;

	}


// 0 1 2
// 3 4 5
// 6 7 8


protected:
	double _v[9];
};


