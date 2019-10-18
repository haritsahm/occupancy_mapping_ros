#ifndef PROPERTIES_H
#define PROPERTIES_H

class Point2d
{
public:
  Point2d()
  {
    x = y = 0;
  }
  Point2d(double x_, double y_)
  {
    x = x_;
    y = y_;
  }

public:
  double x,y;
};

#endif // PROPERTIES_H
