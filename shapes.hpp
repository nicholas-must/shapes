#ifndef __SHAPES_H__
#define __SHAPES_H__

/* Shape defintiions */
#define PI 3.14159265359
#define RAD2DEG(x) ((x) * (180 / PI))
#define DEG2RAD(x) ((x) / (180 / PI))
#define SQR(x) ((x)*(x))

namespace shapes
{
  struct Point2 {float x, y;};
  struct Line2d {Point2 *a, *b;};
  struct Triangle2d {Point2 *a, *b, *c;};
  struct Circle2d {Point2 *centre; float radius;};

  typedef struct _pentacle
  {
    Circle2d *pentacleBase; // The centre and radius of the pentacle
    Point2 outerPoints[5]; // These lie on the base circle
    Line2d lines[5]; // These form the star-shape
    Point2 innerPoints[5]; // These are intersections of the lines of the star
  } Pentacle;

  /* Three-dimensional shape definitions */
  struct Point3 {float x, y, z;};
  struct Line3d {Point3 *a, *b;};
  struct LineEq3d { float a, b, c; };
  struct Triangle3d {Point3 *a, *b, *c;};
  struct Circle3d {Point3 *centre; float radius;};
  typedef Circle3d Sphere;
  struct PlaneEq { float z_x_const, z_x_slope, z_y_const, z_y_slope; };
  
  /* Function definitions */
  //TODO: int get_line_intersect(line3d a, line3d b, point3 *out);
  /* Angle functions */
  int getLineEquation(Line2d *line, float *m, float *c);
  float getAngularDistance(float angle1, float angle2);
  float angleBetweenPoints(Point2 *point1, Point2 *point2);
  float angularDistanceBetweenLines(Point2 *origin, Point2 *line1, Point2 *line2);
  /* Point-inside functions*/
  bool pointInsideCircle(Point2 *point, Circle2d *circle);
  bool pointInsideTriangle(Point2 *point, Triangle2d *triangle);
  namespace s3d {
    bool pointInsideTriangle(Point3 *point, Triangle3d *triangle);
  }
  bool pointInsidePentagon(Point2 *point, Point2 *pentagon[5]);
  int pointInsidePentacleTriangles(Point2 *point, Pentacle *pentacle);
  /* Intersects */
  bool lineIntersectsWithTriangle(Line3d *line, Triangle3d *triangle);
  /* Shape constructors */
  int constructTriangle(Circle2d *triangleBase, Triangle2d *triangle);
  int constructPentacle(Circle2d *pentacleBase, Pentacle *pentacle);

}

#endif // __SHAPES_H__
