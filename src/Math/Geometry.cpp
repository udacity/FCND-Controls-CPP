#include "Common.h"
#include "Geometry.h"

namespace SLR{

V3D PlaneD::Intersect(LineD l) const
{
  V3D a2b = (l._b-l._a);
  
  double denom = _normal.dot(a2b.norm());
  if(denom==0)
  {
    // no intersection
    return V3D::Inf();
  }
  
  double u = _normal.dot(_pt-l._a)/denom;
  return l._a+ a2b*u;
}

V3D PlaneD::Intersect(LineD l, double& distTowardsB) const
{
  V3D a2b = (l._b-l._a);
  
  double denom = _normal.dot(a2b.norm());
  if(denom==0)
  {
    // no intersection
    distTowardsB = numeric_limits<double>::infinity();
    return V3D::Inf();
  }
  
  distTowardsB = _normal.dot(_pt-l._a)/denom;
  return l._a+ a2b*distTowardsB;
}

//http://local.wasp.uwa.edu.au/~pbourke/geometry/3planes/
V3D PlaneD::Intersect(const PlaneD& b, const PlaneD& c) const
{
  double d1 = _pt.dot(_normal);
  double d2 = b._pt.dot(b._normal);
  double d3 = c._pt.dot(c._normal);
  const V3D& N1 = _normal;
  const V3D& N2 = b._normal;
  const V3D& N3 = c._normal;
  return (d1*(N2.cross(N3))+d2*(N3.cross(N1))+d3*(N1.cross(N2)))/(N1.dot(N2.cross(N3)));
}

//http://local.wasp.uwa.edu.au/~pbourke/geometry/planeplane/
//appears to work! :)
LineD PlaneD::Intersect(PlaneD pl) const
{
  V3D dir = _normal.cross(pl._normal);  // direction of line of intersection
  if(dir.mag()==0) return LineD::Invalid();

  double d1 = _pt.dot(_normal);
  double d2 = pl._pt.dot(pl._normal);

  double offDiagonal = _normal.dot(pl._normal); 
  double det = 1 - offDiagonal * offDiagonal; 
  double a = (d1 - d2 * offDiagonal) / det; 
  double b = (d2 - d1 * offDiagonal) / det; 

  V3D A = a * _normal +  b * pl._normal; 
  V3D B = A + dir;
  return LineD(A,B);
}

//http://softsurfer.com/Archive/algorithm_0106/algorithm_0106.htm#Closest%20Point%20of%20Approach
//http://pages.pacificcoast.net/~cazelais/251/distance.pdf
V3D LineD::ClosestPt(const LineD& l) const
{
  V3D u = _b-_a;
  V3D v = l._b-l._a;
  V3D w = _a-l._a;
  double a = u.dot(u);        // always >= 0
  double b = u.dot(v);
  double c = v.dot(v);        // always >= 0
  double d = u.dot(w);
  double e = v.dot(w);
  double D = a*c - b*b;       // always >= 0
  double sc;//, tc;

  // compute the line parameters of the two closest points
  if (D < 1e-15) {         // the lines are almost parallel
    sc = 0.0;
    //tc = (b>c ? d/b : e/c);   // use the largest denominator
  }
  else {
    sc = (b*e - c*d) / D;
    //tc = (a*e - b*d) / D;
  }

  return _a+u*sc;



  /*// get the difference of the two closest points
  Vector   dP = w + (sc * u) - (tc * v);  // = L1(sc) - L2(tc)

  return norm(dP);   // return the closest distance*/

}

V3D LineD::Dist(const V3D& pt) const
{
  V3D v = _b - _a;
  V3D w = pt - _a;

  double c1 = w.dot(v);

  double c2 = v.dot(v);

  double b = c1 / c2;
  V3D closest(_a + b * v);
  return closest.dist(pt);
}

//http://softsurfer.com/Archive/algorithm_0102/algorithm_0102.htm#dist_Point_to_Segment%28%29
V3D LineD::ClosestPt_Segment(const V3D& pt) const
{
  V3D v = _b - _a;
  V3D w = pt - _a;

  double c1 = w.dot(v);
  if ( c1 <= 0 )
      return _a;

  double c2 = v.dot(v);
  if ( c2 <= c1 )
      return _b;

  double b = c1 / c2;
  return (_a + b * v);
}


bool Quad::ClosestPoint_InProjectionOnly(const V3D& pt, V3D& ret, double& signedDist) const
{
  PlaneD plane(pts[0],pts[1],pts[2]);
  V3D p = plane.Project(pt);
  if(!IsInQuad(p)) return false;
  ret = p;
  signedDist = plane.Dist(pt);
  return true;
}





}; // namespace SLR
