#pragma once

// "Absolute threshold" trigger/detector
// Detects if the absolute value of a signal goes below a certain threshold for at least a given time
// and plots the detection point (different color if quiet time met) 

#include "BaseAnalyzer.h"

class WindowThreshold : public BaseAnalyzer
{
public:
  WindowThreshold(string var, float thresh, float minWindow)
  {
    _var = var;
    _thresh = thresh;
    _minWindow = minWindow;
    _lastTime = 0;
    Reset();
  }

  void Reset()
  {
    if (_lastTime != 0)
    {
      if (_active)
      {
        printf("PASS: ABS(%s) was less than %lf for at least %lf seconds\n", _var.c_str(), _thresh, _minWindow);
      }
      else
      {
        printf("FAIL: ABS(%s) was less than %lf for %lf seconds, which was less than %lf seconds\n", _var.c_str(), _thresh, _lastTime- _lastTimeAboveThresh, _minWindow);
      }
    }
    _lastTimeAboveThresh = numeric_limits<float>::infinity();
    _active = false;
  }

  void Update(double time, std::vector<shared_ptr<DataSource> >& sources)
  {
    for (unsigned int j = 0; j < sources.size(); j++)
    {
      float tmp;
      if (sources[j]->GetData(_var, tmp))
      {
        OnNewData((float)time, tmp);
        break;
      }
    }
  }

  void OnNewData(float time, float meas)
  {
    _lastTime = time;
    if (_active)
    {
      return;
    }

    if (_lastTimeAboveThresh == numeric_limits<float>::infinity())
    {
      _lastTimeAboveThresh = time;
    }

    if (fabs(meas) > _thresh)
    {
      _lastTimeAboveThresh = time;
    }

    if ((time - _lastTimeAboveThresh) > _minWindow)
    {
      _active = true;
    }
  }

  // Draws horizontal threshold bands
  // and detection marker/time
  void Draw(float minX, float maxX, float minY, float maxY)
  {
    glColor3f(.1f, .2f, .1f);

    if (_thresh > minY && _thresh < maxY)
    {
      glBegin(GL_LINES);
      glVertex2f(minX, _thresh);
      glVertex2f(maxX, _thresh);
      glEnd();
    }

    if ((-_thresh) > minY && (-_thresh) < maxY)
    {
      glBegin(GL_LINES);
      glVertex2f(minX, -_thresh);
      glVertex2f(maxX, -_thresh);
      glEnd();
    }

    if (_lastTimeAboveThresh == numeric_limits<float>::infinity() || _lastTimeAboveThresh<minX || _lastTimeAboveThresh>maxX)
    {
      return;
    }

    if (_active)
    {
      glColor3f(0, 1, 0);
      glBegin(GL_LINE_STRIP);
      glVertex2f(_lastTimeAboveThresh, CONSTRAIN(_thresh,minY,maxY));
      glVertex2f(_lastTime, CONSTRAIN(_thresh, minY, maxY));
      glVertex2f(_lastTime, CONSTRAIN(-_thresh, minY, maxY));
      glVertex2f(_lastTimeAboveThresh, CONSTRAIN(-_thresh, minY, maxY));
      glVertex2f(_lastTimeAboveThresh, CONSTRAIN(_thresh, minY, maxY));
      glEnd();
    }
    else
    {
      glColor3f(.7f, .1f, .1f);
      if (_thresh > minY && _thresh < maxY)
      {
        glBegin(GL_LINES);
        glVertex2f(minX, _thresh);
        glVertex2f(maxX, _thresh);
        glEnd();
      }
      if (-_thresh > minY && -_thresh < maxY)
      {
        glBegin(GL_LINES);
        glVertex2f(minX, -_thresh);
        glVertex2f(maxX, -_thresh);
        glEnd();
      }
    }
  }

  bool _active;
  string _var;
  float _lastTimeAboveThresh;
  float _thresh, _minWindow;
  float _lastTime;
};