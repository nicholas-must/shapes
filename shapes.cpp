#include "shapes.hpp"

#include <string.h>
#include <stdint.h>
#include <math.h>

//#define _R_DEBUG 1

#if defined(_R_DEBUG)
static int _debug = 1;
#define DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
static int _debug = 0;
#define DEBUG_PRINTF(...) 
#endif

//using namespace shapes;

namespace shapes
{

/**
 * @param[in] line - the line to return the equation constants for
 * @param[out] m - the gradient of line
 * @param[out] c - the y-value value of line at x = 0
 *
 * return - 0 for success, 1 for error
 */
  int getLineEquation(Line2d *line, float *m, float *c)
  {
    // Calculations
    const float rise = ( line->b->y - line->a->y );
    const float run = ( line->b->x - line->a->x );
    const float gradient = ( rise / run );
    const float offset  = line->a->y - ( gradient * line->a->x );
    // Return values
    DEBUG_PRINTF("Equation for line: %f, %f -> %f, %f is y = %.2fx + %.2f \n",
                 line->a->x, line->a->y,
                 line->b->x, line->b->y,
                 gradient, offset);
    *m = gradient;
    *c = offset;
    return 0; // Success
  }

/**
 * @param[in] angle1 The first angle (in degrees)
 * @param[in] angle2 The second angle (in degrees)
 *
 * return A positive number between 0 and 360; a value of >= 180 indicates that angle2 is closest
 * to angle1 counter-clockwise
 */
  float getAngularDistance(float angle1, float angle2)
  {
    float angularDistance = (angle2 - angle1);
    while (angularDistance > 360) angularDistance -= 360;
    while (angularDistance < 0) angularDistance += 360;
    return angularDistance;
  }

  float angleBetweenPoints(Point2 *point1, Point2 *point2)
  {
    // Handle special cases to avoid divide-by-zero
    if (point1->x == point2->x)
    {
      const float theta = (point1->y > point2->y ? 90 : 270);
      return theta;
    }

    // Otherwise just use inverse tan
    const float rise = ( point2->y - point1->y );
    const float run = ( point2->x - point1-> x );
    const float theta = RAD2DEG(atan2(rise, run));
    return theta;
  }

  float angularDistanceBetweenLines(Point2 *origin, Point2 *line1, Point2 *line2)
  {
    DEBUG_PRINTF("origin: %f, %f\tline a:%f, %f\tline b: %f, %f \n",
                 origin->x, origin->y,
                 line1->x, line1->y,
                 line2->x, line2->y);
    const float angle1 = angleBetweenPoints(origin, line1);
    const float angle2 = angleBetweenPoints(origin, line2);
    DEBUG_PRINTF("angle1: %f\t angle2: %f \n", angle1, angle2);
    const float theta = getAngularDistance(angle2, angle1);
    DEBUG_PRINTF("theta: %f \n", theta);
    return theta;
  }

  PlaneEq getPlaneFromTriangle(Triangle3d *triangle)
  {
    // get line equation for triangle points a->b
    Point3 *a = triangle->a, *b = triangle->b;
    // y in terms of x
    float A_y_slope_wrt_x = (b->y - a->y) / (b->x - a->x);
    float A_y_constant_wrt_x = a->y - (A_y_slope_wrt_x * a->x);
    // x in terms of y
    float A_x_slope_wrt_y = (b->x - a->x) / (b->y - a->y);
    float A_x_constant_wrt_y = a->x - (A_x_slope_wrt_y * a->y);
    // z in terms of y
    float A_z_slope_wrt_y = (b->z - a->z) / (b->y - a->y);
    float A_z_constant_wrt_y = a->z - (A_z_slope_wrt_y * a->y);

    // calculate intersections with line A by project point C along the x- and y-axes
    Point3 *p_3 = triangle->c;
    Point3 p_A3_xaxis = {
      A_x_constant_wrt_y + A_x_slope_wrt_y * p_3->y,
      p_3->y,
      A_z_constant_wrt_y + A_z_slope_wrt_y * p_3->y
    };
    float p_A3_yvalue = (A_y_constant_wrt_x + A_y_slope_wrt_x * p_3->x);
    Point3 p_A3_yaxis = {
      p_3->x,
      (p_A3_yvalue),
      A_z_constant_wrt_y + A_z_slope_wrt_y * (p_A3_yvalue)
    };

    // finally calculate the plane relationships between x & z, and y & z
    float z_x_component_slope = 		(p_3->z - p_A3_xaxis.z) / (p_3->x - p_A3_xaxis.x);
    float z_x_component_constant =	p_3->z - (z_x_component_slope * p_3->x);
    float z_y_component_slope = 		(p_3->z - p_A3_yaxis.z) / (p_3->y - p_A3_yaxis.y);
    float z_y_component_constant = 	p_3->z - (z_y_component_slope * p_3->y);

    PlaneEq plane_out = { z_x_component_constant, z_x_component_slope,
                          z_y_component_constant, z_y_component_slope };
    return plane_out;
  }

  Point3 getLineIntersectWithPlane(PlaneEq *plane, Line3d *line)
  {
    // get the line from the plane that is co-planar with the specified line
    Point3 pl_a = {
      line->a->x,
      line->a->y,
      plane->z_x_const + (plane->z_x_slope * line->a->x) + (plane->z_y_slope * line->a->y)
    };
    Point3 pl_b = {
      line->b->x,
      line->b->y,
      plane->z_x_const + (plane->z_x_slope * line->b->x) + (plane->z_y_slope * line->b->y)
    };
    Line3d plane_line = { &pl_a, &pl_b };

    // get line equations for both lines and calculate intersect point
    Point3 *l_a = line->a;
    Point3 *l_b = line->b;
    float l_z_slope_wrt_y = (l_b->z - l_a->z) / (l_b->y - l_a->y);
    float l_z_const_wrt_y = l_a->z - (l_z_slope_wrt_y * l_a->y);
    float pl_z_slope_wrt_y = (pl_b.z - pl_a.z) / (pl_b.y - pl_a.y);
    float pl_z_const_wrt_y = pl_a.z - (pl_z_slope_wrt_y * pl_a.y);
    
    // find x, y where line.z == plane_line.z
    float y_intersect = (pl_z_const_wrt_y - l_z_const_wrt_y) / (l_z_slope_wrt_y - pl_z_slope_wrt_y);
    float x_slope_wrt_y = (l_b->x - l_a->x) / (l_b->y - l_a->y);
    float x_const_wrt_y = l_a->x - (x_slope_wrt_y * l_a->y);
    float x_intersect = x_const_wrt_y + (y_intersect * x_slope_wrt_y);
    float z_intersect = l_z_const_wrt_y + (y_intersect * l_z_slope_wrt_y);

    // return intersect point
    Point3 p_intersect = { x_intersect, y_intersect, z_intersect };
    return p_intersect;
  }

  bool lineIntersectsWithTriangle(Line3d *line, Triangle3d *triangle)
  {
    // define the plane the triangle is on
    PlaneEq plane = getPlaneFromTriangle(triangle);
    // find the intersect point of the line on the place
    Point3 intersect = getLineIntersectWithPlane(&plane, line);
    // check if the point is in the triangle
    return (s3d::pointInsideTriangle(&intersect, triangle));
  }
  
  // Determine if the point is inside of the triangle
  // TODO: Handle line- and point- triangle cases
  bool pointInsideTriangle(Point2 *point, Triangle2d *triangle)
  {
    DEBUG_PRINTF("pointInsideTriangle start \n");
    Triangle2d tempTri;
    tempTri.a = triangle->a;
    tempTri.b = triangle->b;
    tempTri.c = triangle->c;
    // Make sure that the points are clock-wise
    if (180 < angularDistanceBetweenLines(tempTri.a, tempTri.b, tempTri.c))
    {
      Point2 *temp = tempTri.b;
      tempTri.b = tempTri.c;
      tempTri.c = temp;
      DEBUG_PRINTF("pointInsideTriangle: Swapped for clockwise \n");
    }
    // The point must be clockwise of all three points to be 'in the triangle'
    DEBUG_PRINTF("pointInsideTriangle: Checking angle1.. \n");
    const float angle1 = angularDistanceBetweenLines(tempTri.a, tempTri.b, point);
    if (180 < angle1) return false;
    DEBUG_PRINTF("pointInsideTriangle: Checking angle2.. \n");
    const float angle2 = angularDistanceBetweenLines(tempTri.b, tempTri.c, point);
    if (180 < angle2) return false;
    DEBUG_PRINTF("pointInsideTriangle: Checking angle3.. \n");
    const float angle3 = angularDistanceBetweenLines(tempTri.c, tempTri.a, point);
    if (180 < angle3) return false;
    DEBUG_PRINTF("pointInsideTriangle: Point is inside triangle \n");
    return true;
  }

  bool s3d::pointInsideTriangle(Point3 *point, Triangle3d *triangle)
  {
    Point2 p_a = { triangle->a->x, triangle->a->y };
    Point2 p_b = { triangle->b->x, triangle->b->y };
    Point2 p_c = { triangle->b->x, triangle->b->y };
    Triangle2d flat_tri = { &p_a, &p_b, &p_c };
    Point2 flat_p = { point->x, point->y };
    return (pointInsideTriangle(&flat_p, &flat_tri));
  }

  bool pointInsidePentagon(Point2 *point, Point2 *pentagon[5])
  {
    // This function assumes clockwise points as input
    //for (int index = 0; index < 5; ++index)
    for (int index = 0; index < 1; ++index)
    {
      Point2 *pentagonPoint1 = pentagon[index];
      Point2 *pentagonPoint2 = pentagon[( (index + 1) % 5)];
      const float theta = angularDistanceBetweenLines(pentagon[(index + 1) % 5],
                                                      pentagon[index],
                                                      pentagon[(index + 2) % 5]);
      //angularDistanceBetweenLines(pentagonPoint1, pentagonPoint2, point);
      DEBUG_PRINTF("pointInsidePentagon theta=%f \n", theta);
      if (180 < theta) return false;
    }
    return true;
  }

  bool pointInsideCircle(Point2 *point, Circle2d *circle)
  {
    const float h_diff = abs(point->x - circle->centre->x);
    const float v_diff = abs(point->y - circle->centre->y);
    const float distance_from_centre = sqrt((h_diff * h_diff) + (v_diff * v_diff));
    return ( distance_from_centre <= circle->radius );
  }

/**
 * constructs an equilateral trangle based using the centre and radius of the base circle
 */
  int constructTriangle(Circle2d *triangleBase, Triangle2d *triangle)
  {
    if ((0 == triangleBase) || (0 == triangle)) return 1; // Failure

    for (int pointIndex = 0; pointIndex < 3; ++pointIndex)
    {
      Point2 *point = ((0 == pointIndex) ? triangle->a :
                      (1 == pointIndex) ? triangle->b :
                      triangle->c);
      float theta = 90.0 - ( (360.0 / 3.0) * pointIndex);
      while (theta >= 360.0) theta -= 360.0;
      while (theta < 0.0) theta += 360.0;
      point->x = triangleBase->centre->x + cos(DEG2RAD(theta)) * triangleBase->radius;
      point->y = triangleBase->centre->y + sin(DEG2RAD(theta)) * triangleBase->radius;
    }
    return 0; // Success
  }

/**
 * Does the math to construct a pentacle given a centre and a radius
 *
 * @param[in] pentacleBase This is the centre and radius of the pentacle
 * @param[in/out] pentacle Not allocated by function, just filled out
 *
 * return 0 success, 1 fail
 */
  int constructPentacle(Circle2d *pentacleBase, Pentacle *pentacle)
  {
    if ((0 == pentacleBase) || (0 == pentacle)) return 1; // failure

    DEBUG_PRINTF("constructPentacle from base %f, %f (%f) \n",
                 pentacleBase->centre->x,
                 pentacleBase->centre->y,
                 pentacleBase->radius);
  
    // Add points around edge of circle
    for (int outerPointIndex= 0; outerPointIndex< 5; ++outerPointIndex)
    {
      Point2 *point = &pentacle->outerPoints[outerPointIndex];
      float theta = ( (360.0 / 5.0) * outerPointIndex) - 90.0;
      while (theta >= 360.0) theta -= 360.0;
      while (theta < 0.0) theta += 360.0;
      point->x = pentacleBase->centre->x + cos(DEG2RAD(theta)) * pentacleBase->radius;
      point->y = pentacleBase->centre->y + sin(DEG2RAD(theta)) * pentacleBase->radius;
      DEBUG_PRINTF("Pentacle, outer %d: %f degrees, %f, %f \n",
                   outerPointIndex,
                   theta,
                   point->x, point->y);
    }

    // Add lines between the outer points to make a star
    for (int line_index = 0; line_index < 5; ++line_index)
    {
      Line2d *line = &pentacle->lines[line_index];
      line->a = &pentacle->outerPoints[line_index];
      line->b = &pentacle->outerPoints[ ( line_index + 2) % 5 ];
      DEBUG_PRINTF("Pentacle, line %d: %f, %f -> %f, %f \n",
                   line_index,
                   line->a->x, line->a->y,
                   line->b->x, line->b->y);
    }

    // Add the intersections of the lines
    for (int innerPointIndex = 0; innerPointIndex < 5; innerPointIndex++)
    {
      Point2 *innerPoint = &pentacle->innerPoints[innerPointIndex];
      Line2d *l1 = &pentacle->lines[innerPointIndex];
      Line2d *l2 = &pentacle->lines[( (innerPointIndex + 4) % 5)];
      // Get the equations for these two lines
      float m1, c1, m2, c2;
      getLineEquation(l1, &m1, &c1);
      getLineEquation(l2, &m2, &c2);
      // and solve for the intersection
      innerPoint->x = (c1 - c2) / (m2 - m1);
      //innerPoint->y = (m2 / c2) - (m1 / c1);
      //innerPoint->y = ( ( (m1 / m2) + c1 - c2 ) / ( 1 - (m1 / m2) ) );
      //innerPoint->y = ((m2 * c1) - (m1 * c2)) / (m1 + m2);
      innerPoint->y = ((m1 * c2) - (m2 * c1)) / (m1 - m2);

      // Dummy code to check later calcs
      // float theta = 90.0 + (360.0 / 5.0) - ( (360.0 / 5.0) * innerPointIndex);
      // while (theta >= 360.0) theta -= 360.0;
      // while (theta < 0.0) theta += 360.0;
      // innerPoint->x = pentacleBase->centre->x + cos(DEG2RAD(theta)) * pentacleBase->radius * 0.5;
      // innerPoint->y = pentacleBase->centre->y + sin(DEG2RAD(theta)) * pentacleBase->radius * 0.5;
    
      DEBUG_PRINTF("Pentacle, inner: %d: %f, %f \n",
                   innerPointIndex,
                   innerPoint->x, innerPoint->y);
    }
  
    return 0; // success
  }

  int pointInsidePentacleTriangles(Point2 *point, Pentacle *pentacle)
  {
    for (int j = 0; j < 5; ++j)
    {
      Triangle2d pentacleTriangle;
      pentacleTriangle.a = &pentacle->outerPoints[j];
      pentacleTriangle.b = &pentacle->innerPoints[j];
      pentacleTriangle.c = &pentacle->innerPoints[( (j + 4) % 5 )];
      DEBUG_PRINTF("Pentacle triangle %d: %f, %f -> %f, %f -> %f, %f \n",
                   j,
                   pentacleTriangle.a->x, pentacleTriangle.a->y,
                   pentacleTriangle.b->x, pentacleTriangle.b->y,
                   pentacleTriangle.c->x, pentacleTriangle.c->y);
      if (pointInsideTriangle(point, &pentacleTriangle)) return (j + 1);
    }
    return 0; // Not in any triangles
  }

}
