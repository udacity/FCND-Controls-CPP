#pragma once

#include <vector>
#include <map>
using namespace std;
#include "../Utility/FixedQueue.h"

class QuadDynamics;
class DataSource;
class BaseAnalyzer;

class Graph
{
public:
  Graph(const char* name);
  void Reset();
  void Clear();
  void Update(double time, std::vector<shared_ptr<DataSource> >& sources);

  void Draw();
  void AddItem(string path);
  void AddSeries(string path, bool autoColor = true, V3F color = V3F());
  void AddAbsThreshold(string path);
  void AddWindowThreshold(string path);
  bool IsSeriesPlotted(string path);
  void RemoveAllElements();


  struct Series
  {
    Series();
    V3F _color;
    string _yName;
    string _objName, _fieldName;
    FixedQueue<float> x;
    FixedQueue<float> y;
    void Clear()
    {
      x.reset();
      y.reset();
    }
  };

  vector<shared_ptr<BaseAnalyzer> > _analyzers;

  void DrawSeries(Series& s);
  
  vector<Series> _series;
  string _name;
};