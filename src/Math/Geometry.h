// Geometry utilities, super-lightweight-robotics library
// License: BSD-3-clause

#pragma once

#include "Common.h"

#include <vector>
using std::vector;

namespace SLR{

  class LineD;

  // class for dealing with double-precision planes in 3D
  // internally the plane is represented as a normal and a point-on-plane
  class PlaneD
  {
  public:
    PlaneD(V3D pt=V3D(), V3D normal=V3D(0,0,1))
    {
      _pt = pt;
      _normal = normal.norm();
    }
    
    // points should be in COUNTERCLOCKWISE order
    PlaneD(V3D p0ccw, V3D p1ccw, V3D p2ccw)
    {
      _normal = (p1ccw-p0ccw).cross(p2ccw-p0ccw).norm();
      _pt = p0ccw;
    }

    // ABCD format where plane is specified as Ax+By+Cz+D=0
    PlaneD(V3D normal, double d)
    {
      _normal = normal;
      _pt = -normal*d;
    }
    
    double D() const
    {
      return -_pt.dot(_normal);
    }

    // least-squares fit!
    static PlaneD LeastSquaresFit(vector<V3D>& pts);

    // positive = plane normal points *toward* point
    // negative = plane normal points *away* from point
    double Dist(V3D pt) const
    {
      V3D p = pt-_pt; 
      return p.norm().dot(_normal)*p.mag();
    }

    V3D Normal() const    {return _normal;}

    V3D Intersect(const PlaneD& b, const PlaneD& c) const;
    
    V3D Project(V3D pt) const
    {
      V3D p = pt-_pt; 
      return pt - p.norm().dot(_normal)*p.mag()*_normal;
    }

    // given a 3d coordinate, leaves it alone if it's at least 'offset' in front of the plane.
    // Otherwise 'pushes' the input point in the direction orthogonal to the plane until it's 'offset'
    // in front of it and returns that coordinate.
    // (This is useful, for example, for getting a safe return point if a vehicle is too close to a plane)
    V3D PushOut(V3D pt, double offset=0)
    {
      V3D p = pt-_pt; 
      double dist = p.norm().dot(_normal)*p.mag();
      if(dist >= offset) return pt;
      return pt - p.norm().dot(_normal)*(p.mag()+offset)*_normal;
    }

    V3D Intersect(LineD l) const;
    V3D Intersect(LineD l, double& distTowardsB) const;

    // returns a line where two planes intersect
    // direction of line (a towards b) determined by RH rule
    // with *this* plane, and pl (i.e. *this cross pl)
    LineD Intersect(PlaneD pl) const;

    PlaneD Flip() const
    {
      return PlaneD(_pt,-_normal);
    }

    string ToString() const
    {
      char buf[200];
      sprintf_s(buf,200,"pt={%.3lf %.3lf %.3lf} norm={%.3lf %.3lf %.3lf}",
        _pt.x,_pt.y,_pt.z,_normal.x,_normal.y,_normal.z);
      return string(buf);
    }

  protected:
    V3D _normal, _pt;
  };

  class LineD
  {
    friend class PlaneD;
  public:
    LineD(V3D a=V3D(), V3D b=V3D(1,0,0))
    {
      _a = a;
      _b = b;
    }

    static LineD Invalid()
    {
      return LineD(-V3D::Inf(),V3D::Inf());
    }

    V3D ClosestPt_Segment(const V3D& pt) const;  
    V3D ClosestPt(const LineD& l) const;
    V3D Dist(const V3D& pt) const;
    bool IsEndpoint(const V3D& pt) const{ return pt==_a || pt==_b;};

  protected:
    V3D _a, _b;
  };





//////////////////////////////////////////////

// planar, convex quad
class Quad
{
public:
  // point order: CCW about normal
  Quad(){};
  Quad(V3D a, V3D b, V3D c, V3D d)
  {
    pts[0]=a;
    pts[1]=b;
    pts[2]=c;
    pts[3]=d;
  }
  Quad(const vector<V3D> &_pts)
  {
    if(_pts.size()!=4) return;
    for(int i=0;i<4;i++)
    {
      pts[i]=_pts[i];
    }
  }
  Quad(const vector<V3F> &_pts)
  {
    if(_pts.size()!=4) return;
    for(int i=0;i<4;i++)
    {
      pts[i]=_pts[i];
    }
  }
  
  bool IsInQuad(const V3D& pt) const
  {
    const double planarTolerance = 1e-10;
    // check that it's in-plane
    if(fabs(PlaneD(pts[0],pts[1],pts[2]).Dist(pt))>planarTolerance) return false;
    
    // possibly silly algorithm
    V3D v01 = (pts[1]-pts[0]).norm();
    //V3D v02 = (pts[2]-pts[0]).norm();
    V3D v03 = (pts[3]-pts[0]).norm();
    V3D v21 = (pts[1]-pts[2]).norm();
    V3D v23 = (pts[3]-pts[2]).norm();

    V3D v0 = (pt-pts[0]).norm();
    V3D v1 = (pt-pts[1]).norm();
    V3D v2 = (pt-pts[2]).norm();
    V3D v3 = (pt-pts[3]).norm();

    // check that it lies on the right side of v01
    if(v03.cross(v01).dot(v0.cross(v01))<0) return false;

    // check that it lies on the right side of v12
    if((-v01).cross(-v21).dot(v1.cross(-v21))<0) return false;

    // check that it lies on the right side of v23
    if((v21).cross(v23).dot(v2.cross(v23))<0) return false;

    // check that it lies on the right side of v30
    if((-v23).cross(-v03).dot(v3.cross(-v03))<0) return false;
    
    return true;
  }

  // type = 0 if in-plane, 1 if edge, 2 if corner
  V3D ClosestPoint(const V3D& pt, double* signedDist=NULL, unsigned char* type=NULL) const
  {
    PlaneD plane(pts[0],pts[1],pts[2]);
    V3D p = plane.Project(pt);
    if(IsInQuad(p)) 
    {
      if(type!=NULL) *type=0;
      if(signedDist!=NULL) *signedDist=plane.Dist(pt);
      return p;
    }
    LineD lines[4] = {LineD(pts[0],pts[1]),LineD(pts[1],pts[2]),LineD(pts[2],pts[3]),LineD(pts[3],pts[0])};
    double dist = numeric_limits<double>::infinity();
    V3D ret;
    for(int i=0;i<4;i++)
    {
      V3D p = lines[i].ClosestPt_Segment(pt);
      if(p.dist(pt)<dist)
      {
        if(type!=NULL)
        {
					if(!lines[i].IsEndpoint(p))
					{
						*type = 1+((i+3)%4);
					}
					else
					{
						if(pts[i].dist(p) < pts[(i+1)%4].dist(p))
						{
							*type = 5+i;
						}
						else
						{
							*type = 5+ ((i+1)%4);
						}
					}
        }
        if(signedDist!=NULL)
        {
          *signedDist = p.dist(pt);
          if(plane.Dist(pt)<0) *signedDist = -(*signedDist);
        }
        dist = p.dist(pt);
        ret = p;
      }
    }
    return ret;
  }

  bool ClosestPoint_InProjectionOnly(const V3D& pt, V3D& ret, double& signedDist) const;

  V3D pts[4];
};


} //namespace SLR
