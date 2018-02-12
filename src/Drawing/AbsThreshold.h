#pragma once

// "Absolute threshold" trigger/detector
// Detects if the absolute value of a signal goes below a certain threshold for at least a given time
// and plots the detection point (different color if quiet time met) 

class AbsThreshold
{
public:
  AbsThreshold(string var, float thresh, float quietTime)
  {
    _var = var;
    _thresh = thresh;
    _quietTime = quietTime;
    Reset();
  }

  void Reset()
  {
    _lastTimeAboveThresh = numeric_limits<float>::infinity();
    _triggered = false;
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
    if (_triggered)
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

    if ((time - _lastTimeAboveThresh) > _quietTime)
    {
      _triggered = true;
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

    if (_triggered)
    {
      glColor3f(0, 1, 0);
    }
    else
    {
      glColor3f(.2f, .4f, .2f);
    }
    glBegin(GL_LINES);
    glVertex2f(_lastTimeAboveThresh, minY);
    glVertex2f(_lastTimeAboveThresh, maxY);
    glEnd();

    char buf[100];
    sprintf_s(buf, 100, "t_set = %.3lf", _lastTimeAboveThresh);
    DrawStrokeText(buf, _lastTimeAboveThresh + (maxX - minX)*.05f , minY + (maxY - minY) / 2.f, 0, 1.2f, (maxX - minX) / 2.f, (maxY - minY) / 2.f);
  }

  bool _triggered;
  string _var;
  float _lastTimeAboveThresh;
  float _thresh, _quietTime;
};