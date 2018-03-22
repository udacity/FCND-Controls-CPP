#pragma once

class BaseAnalyzer
{
public:

  virtual void Reset() {};
  virtual void Update(double time, std::vector<shared_ptr<DataSource> >& sources) {};
  virtual void Draw(float minX, float maxX, float minY, float maxY) {}
};