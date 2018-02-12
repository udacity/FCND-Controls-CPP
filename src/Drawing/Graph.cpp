#include "Common.h"
#include "Graph.h"
#include "Drawing/DrawingFuncs.h"
#include "Utility/SimpleConfig.h"
#include "Utility/StringUtils.h"
#include "DataSource.h"
#include "ColorUtils.h"
#include "Drawing/AbsThreshold.h"

using namespace SLR;

#define MAX_POINTS 10000

Graph::Graph(const char* name)
{
  _name = name;
  Reset();

}

Graph::Series::Series()
  : x(MAX_POINTS, 0), y(MAX_POINTS, 0)
{

}

void Graph::AddItem(string path)
{
  if (path.find("AbsThreshold(") != string::npos)
  {
    AddAbsThreshold(path.substr(12));
  }
  else
  {
    AddSeries(path);
  }
}

void Graph::AddAbsThreshold(string path)
{
  path = SLR::Trim(path);
  if (path.length() < 4 || path[0] != '(' || path[path.length() - 1] != ')')
  {
    SLR_WARNING1("Malformed AddThreshold command (%s)", path.c_str());
    return;
  }
  path = path.substr(1, path.length() - 2);

  vector<string> args = SLR::Split(path, ',');

  if (args.size()!=3 || args[0]=="" || args[1]=="" || args[2]=="")
  {
    SLR_WARNING1("Malformed AddThreshold command (%s)", path.c_str());
    return;
  }
  
  _absThreshold.reset(new AbsThreshold(args[0], (float)atof(args[1].c_str()), (float)atof(args[2].c_str())));
}

void Graph::AddSeries(string path, bool autoColor, V3F color)
{
	ParamsHandle config = SimpleConfig::GetInstance();
	
	Series newSeries;
  newSeries._yName = path;
  
  newSeries._objName = SLR::LeftOf(newSeries._yName, '.');
  newSeries._fieldName = newSeries._yName.substr(newSeries._objName.size() + 1);

  // If the series is already plotted, then don't add the series again => return
  if (IsSeriesPlotted(path))
  {
    return;
  }

  newSeries._color = color;
  if (autoColor)
  {
    float hue = ((float)_series.size())*30.f;
    newSeries._color = HSVtoRGB(hue + 15.f , 1, 1);
  }
  _series.push_back(newSeries);
}

bool Graph::IsSeriesPlotted(string path)
{
  // Loop through the series vector and check if the field already exists there
  for (unsigned int i = 0; i < _series.size(); i++)
  {

    if (!SLR::ToUpper(path).compare(SLR::ToUpper(_series.at(i)._yName)))
    {
      return true;
    }
  }

  return false;
}

void Graph::RemoveAllSeries()
{
  _series.clear();
}

void Graph::Reset()
{
  if (_series.empty())
  {
    //TODO: temporary while we figure out if graphs should always reload from files, 
    // or if selected graphs should be re-added each time, etc etc..
		ParamsHandle config = SimpleConfig::GetInstance();

    _series.clear();
    while (1)
    {
      char tmp[100];
      sprintf_s(tmp, 100, "%s.Y%d", _name.c_str(), (int)_series.size() + 1);

      string path;
      if (!config->GetString(string(tmp) + ".field", path))
      {
        break;
      }

      V3F color;
      bool specifiedColor = config->GetV3F(string(tmp) + ".color", color);
      AddSeries(path, !specifiedColor, color);
    }
  }
  else
  {
    for (unsigned int i = 0; i < _series.size(); i++)
    {
      _series[i].Clear();
    }
  }
}

void Graph::Clear()
{
  if (_absThreshold)
  {
    _absThreshold->Reset();
  }

  if (_series.empty())
  {
    return;
  }

  for (unsigned int i = 0; i < _series.size(); i++)
  {
    _series[i].Clear();
  }
}

void Graph::Update(double time, std::vector<shared_ptr<DataSource> >& sources)
{
  for (unsigned int i = 0; i < _series.size(); i++)
  {
    for (unsigned int j = 0; j < sources.size(); j++)
    {
      float tmp;
      if (sources[j]->GetData(_series[i]._yName, tmp))
      {
        _series[i].x.push((float)time);
        _series[i].y.push(tmp);
        break;
      }
    } 
  }

  if (_absThreshold)
  {
    _absThreshold->Update(time,sources);
  }
}

void GetRange(FixedQueue<float>& f, float& low, float& high)
{
  low = high = 0;
  if (f.n_meas() == 0) return;
  low = high = f[0];
  for (unsigned int i = 1; i < f.n_meas(); i++)
  {
    low = MIN(low, f[i]);
    high = MAX(high, f[i]);
  }

}

void Graph::DrawSeries(Series& s)
{
  if (s.x.n_meas() < 2 || s.x.n_meas() != s.y.n_meas()) return;

  glColor3f(s._color[0], s._color[1], s._color[2]);

  glBegin(GL_LINE_STRIP);
  for (unsigned int i = 0; i < s.x.n_meas(); i++)
  {
    glVertex2f(s.x[i], s.y[i]);
  }
  glEnd();

}

// Given a data range (r), what is the format string we should use for printing the tick
// labels, how many ticks should there be, and what should the tick labels be?
// This is a rough pragmatic solution that usually produces reasonable results
// 
// Input: low, high -> range of the data of interest
// Returns: string w/ printf format to use for printing the tick labels (e.g. "%.2lf)
// Optional returns:
//    tick: if !null, gets the value at of the 'optimal' tick-tick distance
//	  A: if !null, set to the 'tick' value just below the input range
//	  B: if !null, set to the 'tick' value just above the input range
string GetValueFormat(float low, float high, float* tick, float* A, float* B)
{
  char format[100];

  float range = high - low;
  float T = powf(10.f, floor(log10f(range)));
  if (range / T < 4) T /= 2.f;
  if (range / T > 8) T *= 2.f;

  if (T<1)
  {
    sprintf_s(format, 100, "%%.%dlf", -(int)floor(log10f(T)));
  }
  else
  {
    sprintf_s(format, 100, "%%.0lf");
  }

  if (tick != NULL)		*tick = T;
  if (A != NULL)		*A = low - fmodf(low, T);
  if (B != NULL)		*B = high - fmodf(high, T) + T;
  return format;
}

void Graph::Draw()
{
  if (_series.size() == 0) return;

  // find range
  float lowX = 0, highX = 0, lowY = 0, highY = 0;

  for (unsigned int i = 0; i < _series.size(); i++)
  {
    float tmpLY = lowY, tmpHY = highY;
    GetRange(_series[i].y, tmpLY, tmpHY);
    if (i == 0)
    {
      lowY = tmpLY;
      highY = tmpHY;
    }
    else
    {
      lowY = MIN(lowY, tmpLY);
      highY = MAX(highY, tmpHY);
    }

    float tmpLX = lowX, tmpHX = highX;
    GetRange(_series[i].x, tmpLX, tmpHX);
    if (i == 0)
    {
      lowX = tmpLX;
      highX = tmpHX;
    }
    else
    {
      lowX = MIN(lowX, tmpLX);
      highX = MAX(highX, tmpHX);
    }
  }

  if ((highY - lowY) < 0.001f)
  {
    float mid = (highY + lowY) / 2.f;
    lowY = mid - 0.0005f;
    highY = mid + 0.0005f;
  }

  if ((highX - lowX) < 1.f)
  {
    highX = lowX + 1.f;
  }

  // expand by 10%
  float rangeY = highY - lowY;
  lowY -= rangeY * 0.05f;
  highY += rangeY * 0.05f;

  lowX -= (highX - lowX) * .05f;

  glPushMatrix();

  glScalef(2.f / (highX - lowX), 2.f / (highY - lowY), 1.f);
  glTranslatef(-(highX + lowX) / 2.f, -(highY + lowY) / 2.f, 0.f);

  glColor3f(0.3f, 0.3f, 0.3f);

  glLineWidth(1);
  glBegin(GL_LINES);
  // y=0 line
  if (0 > lowY && 0 < highY)
  {
    glVertex2f(lowX, 0);
    glVertex2f(highX, 0);
  }

  // grid
  glColor3f(0.1f, 0.1f, 0.1f);
  
  float tickYDelta = 0, tickYLow = 0, tickYHigh = 0;
  string tickYFormat = GetValueFormat(lowY, highY, &tickYDelta, &tickYLow, &tickYHigh);
  for (float y = tickYLow; y <= tickYHigh; y += tickYDelta)
  {
    if (y < lowY || y> highY) continue;
    glVertex2f(lowX, y);
    glVertex2f(highX, y);
  }

  glEnd(); // GL_LINES

  if (_absThreshold)
  {
    _absThreshold->Draw(lowX, highX, lowY, highY);
  }

  for (unsigned int i = 0; i < _series.size(); i++)
  {
    DrawSeries(_series[i]);
  }

  // tick labels
  glColor3f(.9f,.9f,.9f);
  for (float y = tickYLow; y <= tickYHigh; y += tickYDelta)
  {
    if (y < (lowY + (highY-lowY)*.03f) || y> (highY - (highY - lowY)*.03f)) continue;
    char buf[100];
    sprintf_s(buf, 100, tickYFormat.c_str(), y);
    DrawStrokeText(buf, lowX + .01f*(highX-lowX), y, 0, 1.2f,(highX-lowX)/3.f, (highY - lowY)/3.f);
  }
  
  glPopMatrix();
  
  for (unsigned int i = 0; i < _series.size(); i++)
  {
    glColor3f(_series[i]._color[0], _series[i]._color[1], _series[i]._color[2]);
    DrawStrokeText(ToLower(_series[i]._yName).c_str(), .4f, .9f - i * .09f, 0, 1.5f);
  }

}
